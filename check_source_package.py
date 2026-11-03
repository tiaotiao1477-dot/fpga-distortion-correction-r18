from pathlib import Path
import hashlib,json,xml.etree.ElementTree as E
r=Path(__file__).resolve().parent
for line in (r/'SHA256SUMS.txt').read_text(encoding='utf-8').splitlines():
 h,n=line.split('  ',1);p=r/n
 assert p.is_file() and hashlib.sha256(p.read_bytes()).hexdigest()==h,n
m=json.loads((r/'SOURCE_MANIFEST.json').read_text(encoding='utf-8'));p=r/m['project'];count=0
for block in E.parse(p).findall('.//inputs'):
 for item in block.iter('item'):
  if item.get('file'):
   f=(p.parent/item.get('file')).resolve()
   assert f.is_relative_to(r) and f.is_file(),item.get('file');count+=1
assert count==m['pdsInputCount']
print('OK: source hashes and',count,'current PDS input references verified')
