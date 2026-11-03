`timescale 1ns / 1ps

// Core-clock latency event joiner and bounded snapshot queue.
module measurement_latency_core_r18 #(
    parameter integer SLOTS = 32,
    parameter integer QUEUE_DEPTH = 64
) (
    input              clk,
    input              rst_n,
    input      [63:0]  ticks,
    input              source_event,
    input      [31:0]  source_id,
    input      [31:0]  sensor_id,
    input              source_time_valid,
    input      [63:0]  source_t0,
    input              last_event,
    input      [31:0]  last_sensor_id,
    input      [63:0]  last_ticks,
    input              commit_event,
    input      [31:0]  commit_source_id,
    input      [15:0]  commit_cfg,
    input              display_event,
    input      [31:0]  display_source_id,
    input      [63:0]  display_ticks,
    input      [31:0]  input_observation_errors,
    input      [31:0]  source_commit_count,
    input      [31:0]  source_drop_count,
    input      [31:0]  result_commit_count,
    input      [31:0]  result_drop_count,
    input      [31:0]  display_repeat_count,
    input      [31:0]  input_frame_count,
    input              control_valid,
    input      [31:0]  control_seq,
    input      [31:0]  control_angle_q16,
    input              control_enable,
    output             control_ready,
    input      [31:0]  gamma_applied_seq,
    input              gamma_applied_identity,
    input              gamma_busy,
    input      [31:0]  command_error,
    input              snapshot_req_toggle_async,
    output reg         snapshot_ack_toggle,
    output reg [1023:0] snapshot_payload
);
    reg run_active;
    reg tracker_invalid;
    reg overflow_sticky;
    reg [31:0] epoch;
    reg [31:0] record_seq;
    reg [31:0] applied_control_seq;
    reg [31:0] applied_angle_q16;
    reg [31:0] unmatched_count;
    reg [31:0] overflow_count;

    reg [SLOTS-1:0] source_valid;
    reg [SLOTS-1:0] source_displayed;
    reg [SLOTS-1:0] source_t0_valid;
    reg [SLOTS-1:0] source_last_valid;
    reg [SLOTS-1:0] source_t1_valid;
    reg [31:0] source_epoch_slot [0:SLOTS-1];
    reg [31:0] source_id_slot [0:SLOTS-1];
    reg [31:0] sensor_id_slot [0:SLOTS-1];
    reg [31:0] angle_slot [0:SLOTS-1];
    reg [63:0] t0_slot [0:SLOTS-1];
    reg [63:0] last_slot [0:SLOTS-1];
    reg [63:0] t1_slot [0:SLOTS-1];
    reg [15:0] cfg_slot [0:SLOTS-1];

    reg [SLOTS-1:0] last_table_valid;
    // Pending camera EOF observations survive until core source allocation.
    // The modulo index is only a slot selector; every lookup checks full ID.
    reg [31:0] last_table_sensor [0:SLOTS-1];
    reg [63:0] last_table_ticks [0:SLOTS-1];

    // Per-slot commit qualification avoids a dynamic full-ID RAM read and
    // broadcast CE path. Each constant slot compares its stored ID and epoch.
    reg [31:0] source_index_tmp;
    wire [SLOTS-1:0] commit_slot_match;
    genvar commit_gen;
    generate
        for (commit_gen = 0; commit_gen < SLOTS; commit_gen = commit_gen + 1) begin : gen_commit_slot_match
            assign commit_slot_match[commit_gen] = source_valid[commit_gen] &&
                (source_epoch_slot[commit_gen] == epoch) &&
                (source_id_slot[commit_gen] == commit_source_id) &&
                !(source_event && (source_index_tmp == commit_gen));
        end
    endgenerate

    reg [31:0] queue_head;
    reg [31:0] queue_tail;
    reg [31:0] queue_count;
    // Per-record words: epoch, sequence, source, sensor, angle, T0, Tlast,
    // T1, T2, and commit configuration (14 words total).  Storage is not reset.
    reg [447:0] queue_record [0:QUEUE_DEPTH-1];

    (* ASYNC_REG = "TRUE" *) reg snapshot_req_meta /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg snapshot_req_sync1 /* synthesis syn_keep=1 syn_preserve=1 */;
    (* ASYNC_REG = "TRUE" *) reg snapshot_req_sync2 /* synthesis syn_keep=1 syn_preserve=1 */;

    wire snapshot_fresh = (snapshot_req_sync2 != snapshot_ack_toggle);
    wire snapshot_pop = snapshot_fresh && (queue_count != 32'd0) && !control_valid;

    reg [31:0] display_index;
    reg display_direct_source;
    reg display_existing_source;
    reg display_duplicate;
    reg display_conflict;
    reg [31:0] display_sensor_value;
    reg [31:0] display_angle_value;
    reg display_t0_valid_value;
    reg [63:0] display_t0_value;
    reg display_last_valid_value;
    reg [63:0] display_last_value;
    reg display_t1_valid_value;
    reg [63:0] display_t1_value;
    reg [15:0] display_cfg_value;
    reg display_context_valid;
    // Only validity is reset. Absolute timestamps and all metadata travel
    // together through the selected-data and order-check stages.
    reg pipe_select_valid;
    reg pipe_select_context_ok;
    reg [447:0] pipe_select_record;
    reg [63:0] pipe_select_observed_ticks;
    reg pipe_checked_valid;
    reg pipe_checked_order_ok;
    reg [447:0] pipe_checked_record;
    wire candidate_record_valid = pipe_checked_valid && pipe_checked_order_ok &&
        run_active && !control_valid && !tracker_invalid &&
        (input_observation_errors == 32'd0);
    wire checked_display_invalid = pipe_checked_valid && !pipe_checked_order_ok;
    wire [447:0] queue_write_record = {pipe_checked_record[447:64],
                                     record_seq, pipe_checked_record[31:0]};
    reg [31:0] unmatched_delta;
    reg [31:0] sensor_index_tmp;
    reg [31:0] last_index_tmp;
    reg last_matches_current_source;
    reg commit_current_match;
    reg candidate_overflow;
    reg candidate_push;
    reg [31:0] q_effective_count;
    reg [31:0] q_next_head;
    reg [31:0] q_next_tail;
    reg [31:0] q_next_count;
    reg [1023:0] snapshot_build;
    reg [31:0] snapshot_word1;
    reg snapshot_has_record;
    reg [31:0] snapshot_epoch_value;
    reg [31:0] snapshot_seq_value;
    reg [31:0] snapshot_angle_value;
    reg snapshot_run_value;
    reg snapshot_invalid_value;
    reg snapshot_overflow_value;
    reg [31:0] snapshot_overflow_count_value;
    reg [31:0] snapshot_unmatched_count_value;
    reg [31:0] snapshot_control_seq_value;
    integer comb_i;
    integer seq_i;

    assign control_ready = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            snapshot_req_meta  <= 1'b0;
            snapshot_req_sync1 <= 1'b0;
            snapshot_req_sync2 <= 1'b0;
        end else begin
            snapshot_req_meta  <= snapshot_req_toggle_async;
            snapshot_req_sync1 <= snapshot_req_meta;
            snapshot_req_sync2 <= snapshot_req_sync1;
        end
    end

    // Resolve display metadata without depending on nonblocking-update order.
    // Same-cycle source, last-pixel, and commit observations are forwarded.
    always @* begin
        display_index = display_source_id % SLOTS;
        source_index_tmp = source_id % SLOTS;
        sensor_index_tmp = sensor_id % SLOTS;
        last_index_tmp = last_sensor_id % SLOTS;
        display_direct_source = source_event &&
                                (source_id == display_source_id);
        display_existing_source = source_valid[display_index] &&
                                  (source_epoch_slot[display_index] == epoch) &&
                                  (source_id_slot[display_index] == display_source_id);
        display_duplicate = !display_direct_source && display_existing_source &&
                            source_displayed[display_index];
        display_conflict = source_event && display_event &&
                           (source_index_tmp == display_index) &&
                           (source_id != display_source_id);

        display_sensor_value = 32'd0;
        display_angle_value = 32'd0;
        display_t0_valid_value = 1'b0;
        display_t0_value = 64'd0;
        display_last_valid_value = 1'b0;
        display_last_value = 64'd0;
        display_t1_valid_value = 1'b0;
        display_t1_value = 64'd0;
        display_cfg_value = 16'd0;

        if (display_direct_source) begin
            display_sensor_value = sensor_id;
            display_angle_value = applied_angle_q16;
            display_t0_valid_value = source_time_valid;
            display_t0_value = source_t0;
            if (last_event && (last_sensor_id == sensor_id)) begin
                display_last_valid_value = 1'b1;
                display_last_value = last_ticks;
            end else if (last_table_valid[sensor_index_tmp] &&
                         (last_table_sensor[sensor_index_tmp] == sensor_id)) begin
                display_last_valid_value = 1'b1;
                display_last_value = last_table_ticks[sensor_index_tmp];
            end
            if (commit_event && (commit_source_id == display_source_id)) begin
                display_t1_valid_value = 1'b1;
                display_t1_value = ticks;
                display_cfg_value = commit_cfg;
            end
        end else if (display_existing_source) begin
            display_sensor_value = sensor_id_slot[display_index];
            display_angle_value = angle_slot[display_index];
            display_t0_valid_value = source_t0_valid[display_index];
            display_t0_value = t0_slot[display_index];
            if (last_event &&
                (last_sensor_id == sensor_id_slot[display_index])) begin
                display_last_valid_value = 1'b1;
                display_last_value = last_ticks;
            end else if (source_last_valid[display_index]) begin
                display_last_valid_value = 1'b1;
                display_last_value = last_slot[display_index];
            end else if (last_table_valid[sensor_id_slot[display_index] % SLOTS] &&
                         (last_table_sensor[sensor_id_slot[display_index] % SLOTS] ==
                          sensor_id_slot[display_index])) begin
                display_last_valid_value = 1'b1;
                display_last_value = last_table_ticks[sensor_id_slot[display_index] % SLOTS];
            end
            if (commit_event && (commit_source_id == display_source_id)) begin
                display_t1_valid_value = 1'b1;
                display_t1_value = ticks;
                display_cfg_value = commit_cfg;
            end else if (source_t1_valid[display_index]) begin
                display_t1_valid_value = 1'b1;
                display_t1_value = t1_slot[display_index];
                display_cfg_value = cfg_slot[display_index];
            end
        end

        display_context_valid = display_direct_source || display_existing_source;
        last_matches_current_source = 1'b0;
        for (comb_i = 0; comb_i < SLOTS; comb_i = comb_i + 1) begin
            if (source_valid[comb_i] && (source_epoch_slot[comb_i] == epoch) &&
                (sensor_id_slot[comb_i] == last_sensor_id) &&
                !(source_event && (source_index_tmp == comb_i)))
                last_matches_current_source = 1'b1;
        end
        if (source_event && (sensor_id == last_sensor_id))
            last_matches_current_source = 1'b1;

        commit_current_match = |commit_slot_match;

        unmatched_delta = 32'd0;
        if (run_active && !control_valid && !tracker_invalid &&
            (input_observation_errors == 32'd0)) begin
            if (source_event && source_valid[source_index_tmp] &&
                !source_displayed[source_index_tmp])
                unmatched_delta = unmatched_delta + 32'd1;

            if (commit_event && !commit_current_match &&
                !(source_event && (source_id == commit_source_id)))
                unmatched_delta = unmatched_delta + 32'd1;

            // A pending EOF cache slot may be replaced only by the latest
            // observation for that modulo slot; count the evicted full ID.
            if (last_event && !last_matches_current_source &&
                last_table_valid[last_index_tmp] &&
                (last_table_sensor[last_index_tmp] != last_sensor_id))
                unmatched_delta = unmatched_delta + 32'd1;

            // Display validity is counted after the order-check stage;
            // source/commit/EOF mismatch counts remain in their event cycle.
        end
    end

    always @* begin
        candidate_overflow = candidate_record_valid &&
                             (queue_count >= QUEUE_DEPTH) && !snapshot_pop;
        candidate_push = candidate_record_valid && !control_valid &&
                         ((queue_count < QUEUE_DEPTH) || snapshot_pop);
        q_effective_count = queue_count - (snapshot_pop ? 32'd1 : 32'd0);
        q_next_head = queue_head;
        q_next_tail = queue_tail;
        q_next_count = q_effective_count + (candidate_push ? 32'd1 : 32'd0);
        if (snapshot_pop)
            q_next_head = (queue_head == QUEUE_DEPTH-1) ? 32'd0 : queue_head + 32'd1;
        if (candidate_push)
            q_next_tail = (queue_tail == QUEUE_DEPTH-1) ? 32'd0 : queue_tail + 32'd1;
    end


    // Stage 1: resolve one display event using the original same-cycle
    // forwarding rules. Data registers have no asynchronous reset load.
    // Stage 2: compare the captured absolute times, never the later live ticks.
    // The existing queue consumes stage 2 on the next edge (stage 3).
    always @(posedge clk) begin
        pipe_select_context_ok <= display_context_valid && !display_conflict &&
            display_t0_valid_value && display_last_valid_value && display_t1_valid_value;
        pipe_select_record <= {
            {16'd0, display_cfg_value},
            display_ticks[63:32], display_ticks[31:0],
            display_t1_value[63:32], display_t1_value[31:0],
            display_last_value[63:32], display_last_value[31:0],
            display_t0_value[63:32], display_t0_value[31:0],
            display_angle_value, display_sensor_value, display_source_id,
            32'd0, epoch
        };
        pipe_select_observed_ticks <= ticks;
        pipe_checked_record <= pipe_select_record;
        pipe_checked_order_ok <= pipe_select_context_ok &&
            (pipe_select_record[223:160] <= pipe_select_record[287:224]) &&
            (pipe_select_record[287:224] <= pipe_select_record[351:288]) &&
            (pipe_select_record[351:288] <= pipe_select_record[415:352]) &&
            (pipe_select_record[415:352] <= pipe_select_observed_ticks);
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_select_valid <= 1'b0;
            pipe_checked_valid <= 1'b0;
        end else if (control_valid || !run_active || tracker_invalid ||
                     (input_observation_errors != 32'd0)) begin
            // A new epoch/error invalidates every in-flight record as well as
            // the scoreboard/queue; no previous epoch may leak into a new run.
            pipe_select_valid <= 1'b0;
            pipe_checked_valid <= 1'b0;
        end else begin
            pipe_select_valid <= display_event && !display_duplicate;
            pipe_checked_valid <= pipe_select_valid;
        end
    end

    // Snapshot assembly overlays current counters and command state onto the
    // immutable per-record fields at the queue head.
    always @* begin
        snapshot_build = 1024'd0;
        snapshot_has_record = (queue_count != 32'd0) && !control_valid;
        snapshot_epoch_value = control_valid ? (epoch + 32'd1) : epoch;
        snapshot_seq_value = snapshot_has_record ? queue_record[queue_head][63:32] :
                             (control_valid ? 32'd0 : record_seq);
        snapshot_angle_value = control_valid ? control_angle_q16 : applied_angle_q16;
        snapshot_run_value = control_valid ? control_enable : run_active;
        snapshot_invalid_value = control_valid ? (input_observation_errors != 32'd0) :
                                 (tracker_invalid || (input_observation_errors != 32'd0));
        snapshot_overflow_value = !control_valid &&
                                  (overflow_sticky || candidate_overflow);
        snapshot_overflow_count_value = control_valid ? 32'd0 :
                                        overflow_count + (candidate_overflow ? 32'd1 : 32'd0);
        snapshot_unmatched_count_value = control_valid ? 32'd0 : unmatched_count + unmatched_delta;
        snapshot_control_seq_value = control_valid ? control_seq : applied_control_seq;

        snapshot_word1 = 32'd0;
        snapshot_word1[0] = snapshot_has_record;
        snapshot_word1[1] = snapshot_run_value;
        snapshot_word1[2] = gamma_applied_identity;
        snapshot_word1[3] = snapshot_invalid_value;
        snapshot_word1[4] = snapshot_overflow_value;
        snapshot_word1[5] = snapshot_has_record;
        snapshot_word1[6] = snapshot_has_record;

        snapshot_build[31:0] = 32'h3631_4D4C;
        snapshot_build[63:32] = {16'd1, snapshot_word1[15:0]};
        snapshot_build[95:64] = snapshot_has_record ? queue_record[queue_head][31:0] : snapshot_epoch_value;
        snapshot_build[127:96] = snapshot_seq_value;
        snapshot_build[159:128] = snapshot_has_record ? queue_record[queue_head][95:64] : 32'd0;
        snapshot_build[191:160] = snapshot_has_record ? queue_record[queue_head][127:96] : 32'd0;
        snapshot_build[223:192] = snapshot_has_record ? queue_record[queue_head][159:128] : snapshot_angle_value;
        snapshot_build[255:224] = snapshot_has_record ? queue_record[queue_head][191:160] : 32'd0;
        snapshot_build[287:256] = snapshot_has_record ? queue_record[queue_head][223:192] : 32'd0;
        snapshot_build[319:288] = snapshot_has_record ? queue_record[queue_head][255:224] : 32'd0;
        snapshot_build[351:320] = snapshot_has_record ? queue_record[queue_head][287:256] : 32'd0;
        snapshot_build[383:352] = snapshot_has_record ? queue_record[queue_head][319:288] : 32'd0;
        snapshot_build[415:384] = snapshot_has_record ? queue_record[queue_head][351:320] : 32'd0;
        snapshot_build[447:416] = snapshot_has_record ? queue_record[queue_head][383:352] : 32'd0;
        snapshot_build[479:448] = snapshot_has_record ? queue_record[queue_head][415:384] : 32'd0;
        snapshot_build[511:480] = 32'd125000000;
        snapshot_build[543:512] = source_commit_count;
        snapshot_build[575:544] = source_drop_count;
        snapshot_build[607:576] = result_commit_count;
        snapshot_build[639:608] = result_drop_count;
        snapshot_build[671:640] = display_repeat_count;
        snapshot_build[703:672] = snapshot_overflow_count_value;
        snapshot_build[735:704] = snapshot_unmatched_count_value;
        snapshot_build[767:736] = input_observation_errors;
        snapshot_build[799:768] = snapshot_has_record ? queue_record[queue_head][447:416] : 32'd0;
        snapshot_build[831:800] = snapshot_control_seq_value;
        snapshot_build[863:832] = gamma_applied_seq;
        snapshot_build[895:864] = {gamma_busy, 30'd0, gamma_applied_identity};
        snapshot_build[927:896] = ticks[31:0];
        snapshot_build[959:928] = ticks[63:32];
        snapshot_build[991:960] = input_frame_count;
        snapshot_build[1023:992] = command_error;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_active <= 1'b0;
            tracker_invalid <= 1'b0;
            overflow_sticky <= 1'b0;
            epoch <= 32'd0;
            record_seq <= 32'd0;
            applied_control_seq <= 32'd0;
            applied_angle_q16 <= 32'd0;
            unmatched_count <= 32'd0;
            overflow_count <= 32'd0;
            queue_head <= 32'd0;
            queue_tail <= 32'd0;
            queue_count <= 32'd0;
            snapshot_ack_toggle <= 1'b0;
            snapshot_payload <= 1024'd0;
            source_valid <= {SLOTS{1'b0}};
            source_displayed <= {SLOTS{1'b0}};
            source_t0_valid <= {SLOTS{1'b0}};
            source_last_valid <= {SLOTS{1'b0}};
            source_t1_valid <= {SLOTS{1'b0}};
            last_table_valid <= {SLOTS{1'b0}};
        end else begin
            if (snapshot_fresh) begin
                snapshot_payload <= snapshot_build;
                snapshot_ack_toggle <= snapshot_req_sync2;
            end

            if (control_valid) begin
                epoch <= epoch + 32'd1;
                record_seq <= 32'd0;
                applied_control_seq <= control_seq;
                applied_angle_q16 <= control_angle_q16;
                run_active <= control_enable;
                tracker_invalid <= (input_observation_errors != 32'd0);
                overflow_sticky <= 1'b0;
                unmatched_count <= 32'd0;
                overflow_count <= 32'd0;
                queue_head <= 32'd0;
                queue_tail <= 32'd0;
                queue_count <= 32'd0;
                source_valid <= {SLOTS{1'b0}};
                source_displayed <= {SLOTS{1'b0}};
                source_t0_valid <= {SLOTS{1'b0}};
                source_last_valid <= {SLOTS{1'b0}};
                source_t1_valid <= {SLOTS{1'b0}};
                last_table_valid <= {SLOTS{1'b0}};
            end else begin
                if (input_observation_errors != 32'd0)
                    tracker_invalid <= 1'b1;

                if (run_active && !tracker_invalid &&
                    (input_observation_errors == 32'd0)) begin
                    if (source_event) begin
                        source_valid[source_index_tmp] <= 1'b1;
                        source_displayed[source_index_tmp] <= display_event &&
                                                               (display_source_id == source_id);
                        source_epoch_slot[source_index_tmp] <= epoch;
                        source_id_slot[source_index_tmp] <= source_id;
                        sensor_id_slot[source_index_tmp] <= sensor_id;
                        angle_slot[source_index_tmp] <= applied_angle_q16;
                        source_t0_valid[source_index_tmp] <= source_time_valid;
                        t0_slot[source_index_tmp] <= source_t0;
                        if (last_event && (last_sensor_id == sensor_id)) begin
                            source_last_valid[source_index_tmp] <= 1'b1;
                            last_slot[source_index_tmp] <= last_ticks;
                        end else if (last_table_valid[sensor_index_tmp] &&
                                     (last_table_sensor[sensor_index_tmp] == sensor_id)) begin
                            // Consume only an exact sensor-ID match from an
                            // earlier EOF; stale/mismatched cache entries are
                            // never used as this source's Tlast.
                            source_last_valid[source_index_tmp] <= 1'b1;
                            last_slot[source_index_tmp] <= last_table_ticks[sensor_index_tmp];
                        end else begin
                            source_last_valid[source_index_tmp] <= 1'b0;
                        end
                        if (last_table_valid[sensor_index_tmp] &&
                            (last_table_sensor[sensor_index_tmp] == sensor_id))
                            last_table_valid[sensor_index_tmp] <= 1'b0;
                        if (commit_event && (commit_source_id == source_id)) begin
                            source_t1_valid[source_index_tmp] <= 1'b1;
                            t1_slot[source_index_tmp] <= ticks;
                            cfg_slot[source_index_tmp] <= commit_cfg;
                        end else begin
                            source_t1_valid[source_index_tmp] <= 1'b0;
                        end
                    end

                    if (last_event &&
                        !(source_event && (sensor_id == last_sensor_id))) begin
                        for (seq_i = 0; seq_i < SLOTS; seq_i = seq_i + 1) begin
                            if (source_valid[seq_i] && (source_epoch_slot[seq_i] == epoch) &&
                                (sensor_id_slot[seq_i] == last_sensor_id) &&
                                !(source_event && (source_index_tmp == seq_i))) begin
                                source_last_valid[seq_i] <= 1'b1;
                                last_slot[seq_i] <= last_ticks;
                            end
                        end
                        if (!last_matches_current_source) begin
                            last_table_valid[last_index_tmp] <= 1'b1;
                            last_table_sensor[last_index_tmp] <= last_sensor_id;
                            last_table_ticks[last_index_tmp] <= last_ticks;
                        end
                    end

                    if (commit_event) begin
                        for (seq_i = 0; seq_i < SLOTS; seq_i = seq_i + 1) begin
                            if (commit_slot_match[seq_i]) begin
                                source_t1_valid[seq_i] <= 1'b1;
                                t1_slot[seq_i] <= ticks;
                                cfg_slot[seq_i] <= commit_cfg;
                            end
                        end
                    end

                    if (display_event && !display_direct_source &&
                        display_existing_source && !display_conflict)
                        source_displayed[display_index] <= 1'b1;

                    if ((unmatched_delta != 32'd0) || checked_display_invalid)
                        unmatched_count <= unmatched_count + unmatched_delta +
                            (checked_display_invalid ? 32'd1 : 32'd0);

                    if (candidate_record_valid)
                        record_seq <= record_seq + 32'd1;
                    if (candidate_overflow) begin
                        overflow_count <= overflow_count + 32'd1;
                        overflow_sticky <= 1'b1;
                    end
                    if (candidate_push) begin
                        queue_record[queue_tail] <= queue_write_record;
                    end
                end

                if (run_active && !tracker_invalid &&
                    (input_observation_errors == 32'd0)) begin
                    if (snapshot_pop || candidate_push) begin
                        queue_head <= q_next_head;
                        queue_tail <= q_next_tail;
                        queue_count <= q_next_count;
                    end
                end else begin
                    // Queue reads remain useful while paused; event writes are
                    // disabled with run_active, but a pending snapshot drains.
                    if (snapshot_pop) begin
                        queue_head <= q_next_head;
                        queue_count <= queue_count - 32'd1;
                    end
                end
            end
        end
    end
endmodule
