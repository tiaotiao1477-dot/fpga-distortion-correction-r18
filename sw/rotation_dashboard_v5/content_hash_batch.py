"""Transaction-local full-content SHA-256 prefetch for the qualified sender."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
import os
from pathlib import Path
import time

_CHUNK_BYTES = 1024 * 1024
_MAX_WORKERS = 4


def _signature(path: Path) -> tuple[int, int, int, int, int]:
    stat = path.stat()
    return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)


class _ReadLock:
    def __init__(self, handle=None, stream=None):
        self.handle = handle
        self.stream = stream

    def close(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None
        if self.handle is not None:
            import ctypes
            from ctypes import wintypes
            close_handle = ctypes.WinDLL('kernel32', use_last_error=True).CloseHandle
            close_handle.argtypes = [wintypes.HANDLE]
            close_handle.restype = wintypes.BOOL
            handle = self.handle
            if not close_handle(handle):
                raise ctypes.WinError(ctypes.get_last_error())
            self.handle = None


def _open_read_locked(path: Path):
    """Keep a raw Windows read handle that denies concurrent writes/deletes."""
    if os.name != 'nt':
        return _ReadLock(stream=path.open('rb', buffering=0))
    import ctypes
    from ctypes import wintypes

    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    create_file = kernel.CreateFileW
    create_file.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                            wintypes.LPVOID, wintypes.DWORD, wintypes.DWORD,
                            wintypes.HANDLE]
    create_file.restype = wintypes.HANDLE
    handle = create_file(str(path), 0x80000000, 0x00000001, None, 3, 0x08000000, None)
    if handle == ctypes.c_void_p(-1).value:
        raise ctypes.WinError(ctypes.get_last_error())
    return _ReadLock(handle=handle)


def _fd_signature(stream) -> tuple[int, int, int, int]:
    stat = os.fstat(stream.fileno())
    return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)


def _hash_one(path: Path, lock: _ReadLock) -> tuple[str, tuple[int, int, int, int, int], int]:
    before = _signature(path)
    digest = hashlib.sha256()
    total = 0
    # The raw lock handle prevents replacement or writes; this short-lived CRT
    # stream is one of at most four readers and provides fstat before/after.
    with path.open('rb', buffering=0) as stream:
        opened = _fd_signature(stream)
        if opened != before[:4]:
            raise RuntimeError(f'File identity changed before hashing: {path}')
        while True:
            block = stream.read(_CHUNK_BYTES)
            if not block:
                break
            total += len(block)
            digest.update(block)
        after_fd = _fd_signature(stream)
    after_path = _signature(path)
    if before[:4] != after_fd or before != after_path or total != before[2]:
        raise RuntimeError(f'File changed while hashing: {path}')
    return digest.hexdigest(), before, total



class HashBatch:
    """Re-read and hash every provided path, retaining digests for one send only."""

    def __init__(self, paths, *, workers: int = _MAX_WORKERS,
                 chunk_size: int = _CHUNK_BYTES):
        if not 1 <= workers <= _MAX_WORKERS:
            raise ValueError(f'workers must be between 1 and {_MAX_WORKERS}')
        if chunk_size != _CHUNK_BYTES:
            raise ValueError(f'chunk_size is fixed at {_CHUNK_BYTES} bytes')
        self.max_workers = workers
        self.chunk_size = chunk_size
        self.wall_seconds = 0.0
        self.cpu_seconds = 0.0
        self.bytes_hashed = 0
        self._digests: dict[str, str] = {}
        self._signatures: dict[str, tuple[int, int, int, int, int]] = {}
        self._locks = {}
        self._closed = False
        normalized = {}
        for value in paths:
            path = Path(value).resolve(strict=True)
            normalized[str(path)] = path
        self.file_count = len(normalized)
        start_wall = time.perf_counter()
        start_cpu = time.process_time()
        try:
            for key, path in normalized.items():
                self._locks[key] = _open_read_locked(path)
            worker_count = min(workers, max(1, self.file_count))
            with ThreadPoolExecutor(max_workers=worker_count) as pool:
                results = pool.map(lambda item: _hash_one(*item),
                                   ((path, self._locks[key]) for key, path in normalized.items()))
                for key, result in zip(normalized, results):
                    digest, signature, byte_count = result
                    self._digests[key] = digest
                    self._signatures[key] = signature
                    self.bytes_hashed += byte_count
        except BaseException as exc:
            self._digests.clear()
            self._signatures.clear()
            self.close()
            if isinstance(exc, RuntimeError):
                raise
            raise RuntimeError('File changed or could not be locked while hashing transaction') from exc
        finally:
            self.wall_seconds = time.perf_counter() - start_wall
            self.cpu_seconds = time.process_time() - start_cpu

    def sha(self, value) -> str:
        path = Path(value).resolve(strict=True)
        key = str(path)
        digest = self._digests.get(key)
        if digest is None:
            raise KeyError(f'Path was not fully hashed in this transaction: {path}')
        if self._closed or _signature(path) != self._signatures[key]:
            raise RuntimeError(f'File changed after hashing or transaction lock was released: {path}')
        return digest

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for lock in self._locks.values():
            try:
                lock.close()
            except OSError:
                pass
        self._locks.clear()

    def report(self) -> dict:
        return {'fileCount': self.file_count, 'bytesHashed': self.bytes_hashed,
                'chunkBytes': self.chunk_size, 'workers': min(self.max_workers, max(1, self.file_count)),
                'wallSeconds': self.wall_seconds, 'processCpuSeconds': self.cpu_seconds,
                'allFilesFullyStreamHashed': True}


