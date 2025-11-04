% dtn_relay_realistic.m
% Realistic DTN-like Store-and-Forward Relay Simulation
% - Fragmentation & retransmit (CFDP-like)
% - Friis link budget -> SNR -> data-rate
% - Relay stores bundles and forwards when contact available
% - Outputs: SNR timeline, buffer backlog, throughput, latency CDF
%
% Units: SI (m, s, Hz, W)
clear; close all; clc;

%% ----------------------------- CONFIG ---------------------------------
% Simulation time
Tsim = 6*3600;        % 6 hours
dt = 1;               % time step (s)
t = 0:dt:Tsim; N = length(t);

% Bundle generation (primary)
bundle_rate = 0.02;   % bundles per sec (1 every 50s -> 0.02 Hz)
avg_bundle_bytes = 50e3; % average bundle size 50 KB
bundle_jitter = 0.3;  % +/- fraction jitter on generation interval
priority_levels = [1 2]; % 1 highest, 2 normal

% Fragmentation
MTU = 1500;           % bytes per fragment (typical)
frag_overhead = 50;   % bytes per fragment (headers)
max_retransmissions = 6;

% Buffers
primary_buf_max = 200e6;  % 200 MB
relay_buf_max   = 1000e6; % 1 GB

% Radio / physical layer parameters
c = 3e8;
f = 8e9;              % 8 GHz
lambda = c / f;
k_B = 1.38064852e-23;
T_sys = 400;          % system noise temp K
B = 500e3;            % bandwidth [Hz]
No = k_B * T_sys;

% Tx/Rx (use realistic dBi then convert to linear)
Pt_dBW = 10*log10(20);      % Tx power ~20 W -> in dBW (we'll compute linear anyway)
Pt = 20;                    % W
Gt_dBi = 18; Gt = 10^(Gt_dBi/10); % ~63 linear
Gr_relay_dBi = 22; Gr_relay = 10^(Gr_relay_dBi/10); % high gain on relay
Gr_gs_dBi = 28; Gr_gs = 10^(Gr_gs_dBi/10);

misc_loss_dB = 2; misc_loss = 10^(misc_loss_dB/10);

% coding and modulation
codingEfficiency = 0.7; % spectral efficiency factor (FEC overhead & real modem)
bits_per_symbol = log2(2); % BPSK baseline; but capacity formula used

% Contact geometry (sinusoidal distances approximating periodic visibility)
d_pr_mean = 2200e3; d_pr_amp = 1100e3; period_pr = 3600; % one-hour-ish period
d_rg_mean = 900e3;  d_rg_amp = 400e3;  period_rg = 2700;

% SNR threshold to declare usable link (linear)
min_snr_linear = 10^(5/10); % 5 dB minimum

% random seed
rng(42);

%% ------------------------- INITIALIZE STATE ---------------------------
primary_queue = {}; % cell array of bundles
relay_queue = {};   % cell array of bundles at relay
primary_bytes = 0;
relay_bytes = 0;

% For each bundle: struct fields:
% id, size, genTime, priority, fragments (struct array), status ('queued','delivered','dropped'), deliverTime

next_bundle_time = 0;
bundle_counter = 0;

% Logging
primary_buf_log = zeros(N,1);
relay_buf_log = zeros(N,1);
pr_snr_db = zeros(N,1);
rg_snr_db = zeros(N,1);
tx_pr_log = zeros(N,1); % bytes sent primary->relay this step
tx_relay_log = zeros(N,1); % bytes sent relay->gs
delivered_bundles = [];

%% -------------------- Helper nested functions -------------------------
% compute Friis received power (linear)
friis_recv = @(Pt,W_Gt,W_Gr,d) Pt * W_Gt * W_Gr * (lambda^2) / ((4*pi*d)^2 * misc_loss);

% compute capacity (bytes/s) using Shannon approx and coding efficiency
capacity_bytes = @(snr_lin) max(0, codingEfficiency * B * log2(1 + snr_lin) / 8);

% BER approx from Eb/N0 using BPSK erfc
ber_from_snr = @(snr_lin, Rb) (0.5 * erfc(sqrt(max(snr_lin * (B/Rb), 0)))); 
% Note: here we approximate Eb/N0 ~ SNR * (B/Rb); for fragment-level Rb ~ dataRate_bps

%% ---------------------- MAIN SIMULATION LOOP --------------------------
for k = 1:N
    curT = t(k);
    % --- generate bundles at primary according to bundle_rate (jittered) ---
    if curT >= next_bundle_time
        bundle_counter = bundle_counter + 1;
        bsize = avg_bundle_bytes * (1 + bundle_jitter*(2*rand-1));
        priority = 2; if rand < 0.15, priority = 1; end % some high priority
        % create fragments
        num_frags = ceil(bsize / (MTU - frag_overhead));
        fragments = repmat(struct('size',[], 'sentAttempts',0, 'lastSent', -Inf, 'acked', false), num_frags, 1);
        % fill fragment sizes
        remaining = bsize;
        for fidx = 1:num_frags
            pay = min(MTU - frag_overhead, remaining);
            fragments(fidx).size = pay + frag_overhead;
            fragments(fidx).sentAttempts = 0;
            fragments(fidx).acked = false;
        remaining = remaining - pay;
        end
        bundle.id = bundle_counter;
        bundle.size = bsize;
        bundle.genTime = curT;
        bundle.priority = priority;
        bundle.fragments = fragments;
        bundle.status = 'queued';
        bundle.deliverTime = NaN;
        % enqueue at end
        primary_queue{end+1} = bundle;
        primary_bytes = primary_bytes + bsize;
        % next bundle time
        next_bundle_time = curT + max(5, (1/bundle_rate)*(1 + 0.2*(2*rand-1))); % avoid too small
    end

    % --- compute distances and SNRs ---
    d_pr = d_pr_mean + d_pr_amp * sin(2*pi*curT/period_pr);
    d_rg = d_rg_mean + d_rg_amp * sin(2*pi*(curT+300)/period_rg); % phase offset
    % Received powers
    Prx_pr = friis_recv(Pt, Gt, Gr_relay, d_pr);
    Prx_rg = friis_recv(Pt, Gt, Gr_gs, d_rg); % assume relay uses similar Pt; simplification
    snr_pr = Prx_pr / (No * B);
    snr_rg = Prx_rg / (No * B);
    pr_snr_db(k) = 10*log10(max(snr_pr, 1e-20));
    rg_snr_db(k) = 10*log10(max(snr_rg, 1e-20));
    % link up
    link_pr_up = (d_pr <= 6e6) && (snr_pr >= min_snr_linear);
    link_rg_up = (d_rg <= 6e6) && (snr_rg >= min_snr_linear);

    % --- PRIMARY -> RELAY: send fragments when link up ---
    tx_pr = 0;
    if link_pr_up && ~isempty(primary_queue)
        % capacity in bytes this dt
        cap = floor(capacity_bytes(snr_pr) * dt);
        % Send top-priority-first FIFO within priority (1 then 2)
        % Build sorted index by priority then genTime
        if ~isempty(primary_queue)
            prios = cellfun(@(x) x.priority, primary_queue);
            genTimes = cellfun(@(x) x.genTime, primary_queue);
            % sort priority asc (1 high), then genTime asc
            [~, idxOrder] = sortrows([prios' genTimes'] , [1 2]);
            primary_queue = primary_queue(idxOrder);
        end
        i = 1;
        while cap > 0 && i <= length(primary_queue)
            b = primary_queue{i};
            % find next unacked fragment
            fragIdx = find(~[b.fragments.acked], 1, 'first');
            if isempty(fragIdx)
                % all fragments acked locally (shouldn't happen before relay ack) -> move bundle
                i = i + 1; continue;
            end
            frag = b.fragments(fragIdx);
            % if this fragment was recently sent, allow retransmit if last attempt older than 1s
            if frag.sentAttempts > 0 && (curT - frag.lastSent) < 1
                % skip to next bundle
                i = i + 1; continue;
            end
            if frag.size <= cap
                % send fragment: compute fragment-level BER and loss
                dataRate_bps = max(1, capacity_bytes(snr_pr)*8); % approx bps
                EbN0 = snr_pr * (B / dataRate_bps); % approx
                BER = 0.5 * erfc(sqrt(max(EbN0,0)));
                p_loss = 1 - (1 - BER)^(frag.size*8);
                frag.sentAttempts = frag.sentAttempts + 1;
                frag.lastSent = curT;
                % store back
                b.fragments(fragIdx) = frag;
                primary_queue{i} = b;
                % If delivered to relay (success)
                if rand > p_loss
                    % append fragment to relay queue representation
                    % we will store bundle-level structure at relay and mark frag as received
                    % mark fragment as acked at primary (we emulate ACK from relay when it receives)
                    b.fragments(fragIdx).acked = true;
                    b.fragments(fragIdx).lastSent = curT;
                    primary_queue{i} = b;
                    % add to relay partial storage:
                    % find same bundle in relay queue, else create
                    ridx = find(cellfun(@(x) x.id == b.id, relay_queue), 1);
                    if isempty(ridx)
                        % create relay copy
                        rcopy = b;
                        % but reset sentAttempts on fragments, and mark which fragments received
                        for f2 = 1:length(rcopy.fragments)
                            rcopy.fragments(f2).sentAttempts = 0;
                            rcopy.fragments(f2).acked = false;
                            rcopy.fragments(f2).lastSent = -Inf;
                        end
                        % mark the received frag
                        rcopy.fragments(fragIdx).acked = true;
                        relay_queue{end+1} = rcopy;
                    else
                        % update existing
                        rcopy = relay_queue{ridx};
                        rcopy.fragments(fragIdx).acked = true;
                        relay_queue{ridx} = rcopy;
                    end
                    % update primary bytes accounting: consider fragment payload moved to relay (so reduce primary_bytes by frag payload)
                    primary_bytes = max(0, primary_bytes - (frag.size - frag_overhead));
                    tx_pr = tx_pr + frag.size;
                    cap = cap - frag.size;
                else
                    % lost in the air, leave fragment unacked for retransmit
                    frag.sentAttempts = frag.sentAttempts; % already incremented
                    b.fragments(fragIdx) = frag;
                    primary_queue{i} = b;
                    % deduct capacity (we still used the air time)
                    tx_pr = tx_pr + frag.size;
                    cap = cap - frag.size;
                end
                % if fragment reached max retransmissions -> drop fragment (and eventually bundle)
                if frag.sentAttempts >= max_retransmissions
                    % mark bundle as dropped at primary
                    primary_bytes = max(0, primary_bytes - b.size);
                    primary_queue(i) = []; % remove bundle
                    continue; % do not increment i (since array shifted)
                end
            else
                % not enough cap to send this fragment now -> break
                break;
            end
            % if all fragments acked at primary (i.e., they were forwarded to relay)
            if all([primary_queue{i}.fragments.acked])
                % remove bundle from primary queue (relay now stores)
                primary_queue(i) = [];
                continue;
            end
            % otherwise, continue with same bundle if cap remains
        end
    end
    tx_pr_log(k) = tx_pr;

    % --- RELAY internal: check for bundle completion (all fragments received) ---
    % A bundle is deliverable to GS only when relay has all fragments (marked acked)
    % We'll forward fragments from relay to GS similarly (with retransmits)
    tx_rg = 0;
    if ~isempty(relay_queue)
        % for each relay bundle ensure status
        for ri = 1:length(relay_queue)
            rb = relay_queue{ri};
            if strcmp(rb.status,'delivered'), continue; end
            % if all fragments present (acked true) then eligible for forward
            if all([rb.fragments.acked])
                % forward fragments to GS when link up
                if link_rg_up
                    cap2 = floor(capacity_bytes(snr_rg) * dt);
                    fidx = find(~[rb.fragments.acked]==0, 1, 'first'); %#ok<FNDSB>
                    % Actually fragments in rb.fragments are marked acked when received from primary,
                    % for relay->GS we need to send them and wait ACK from GS on fragment-level.
                    % We'll treat 'acked' here as 'present at relay', and use another field 'gsAcked' to track GS acceptance.
                    % So augment structure if not already
                    if ~isfield(rb.fragments, 'gsAcked')
                        for fi=1:length(rb.fragments)
                            rb.fragments(fi).gsAcked = false;
                            rb.fragments(fi).sendAttempts = 0;
                            rb.fragments(fi).gsLastSent = -Inf;
                        end
                    end
                    % send fragments sequentially
                    for fi = 1:length(rb.fragments)
                        if rb.fragments(fi).gsAcked, continue; end
                        % allow retransmit spacing: don't resend same frag within 1s
                        if rb.fragments(fi).sendAttempts > 0 && (curT - rb.fragments(fi).gsLastSent) < 1
                            continue;
                        end
                        fragsize = rb.fragments(fi).size;
                        if fragsize <= cap2
                            % compute BER on relay->GS link
                            dataRate_bps2 = max(1, capacity_bytes(snr_rg)*8);
                            EbN0_rg = snr_rg * (B / dataRate_bps2);
                            BER_rg = 0.5 * erfc(sqrt(max(EbN0_rg,0)));
                            p_loss_rg = 1 - (1 - BER_rg)^(fragsize*8);
                            rb.fragments(fi).sendAttempts = rb.fragments(fi).sendAttempts + 1;
                            rb.fragments(fi).gsLastSent = curT;
                            % if not lost
                            if rand > p_loss_rg
                                rb.fragments(fi).gsAcked = true;
                                rb.fragments(fi).gsLastSent = curT;
                                % record tx bytes
                                tx_rg = tx_rg + fragsize;
                                cap2 = cap2 - fragsize;
                            else
                                % lost, consume cap but not ack
                                tx_rg = tx_rg + fragsize;
                                cap2 = cap2 - fragsize;
                            end
                            % if too many attempts drop
                            if rb.fragments(fi).sendAttempts >= max_retransmissions
                                % drop bundle at relay
                                relay_bytes = max(0, relay_bytes - rb.size);
                                relay_queue(ri) = []; % remove
                                break;
                            end
                        else
                            % not enough capacity to send this fragment now
                            break;
                        end
                    end
                    % save updated rb back
                    if ri <= length(relay_queue) % ensure not removed
                        relay_queue{ri} = rb;
                    end
                end
            end
            % if all gsAcked then bundle delivered -> record
            if isfield(rb.fragments, 'gsAcked') && all([rb.fragments.gsAcked])
                rb.status = 'delivered';
                rb.deliverTime = curT;
                delivered_bundles = [delivered_bundles; struct('id', rb.id, 'genTime', rb.genTime, 'deliverTime', rb.deliverTime, 'size', rb.size)]; %#ok<AGROW>
                % remove from relay queue
                relay_bytes = max(0, relay_bytes - rb.size);
                relay_queue(ri) = []; % remove
            end
        end
    end
    tx_relay_log(k) = tx_rg;

    % --- update primary_bytes and relay_bytes (for plotting) ---
    % recalc totals
    primary_bytes = sum(cellfun(@(x) x.size, primary_queue));
    relay_bytes = sum(cellfun(@(x) x.size, relay_queue));
    primary_buf_log(k) = primary_bytes;
    relay_buf_log(k) = relay_bytes;

    % --- enforce buffer limits: drop oldest low-priority bundles if overflow ---
    if primary_bytes > primary_buf_max
        % drop oldest (end of queue) until fit
        while primary_bytes > primary_buf_max && ~isempty(primary_queue)
            dropped = primary_queue{1};
            primary_queue(1) = [];
            primary_bytes = sum(cellfun(@(x) x.size, primary_queue));
        end
    end
    if relay_bytes > relay_buf_max
        while relay_bytes > relay_buf_max && ~isempty(relay_queue)
            relay_queue(1) = [];
            relay_bytes = sum(cellfun(@(x) x.size, relay_queue));
        end
    end
end % end time loop

%% -------------------- POSTPROCESS & PLOTS -----------------------------
% Delivered latency
if isempty(delivered_bundles)
    disp('No bundles delivered. Try increasing sim time or improving links.');
else
    lat = [delivered_bundles.deliverTime] - [delivered_bundles.genTime];
    fprintf('Delivered bundles: %d\n', length(delivered_bundles));
    fprintf('Median latency: %.1f s, 90th percentile: %.1f s\n', median(lat), prctile(lat,90));
    %figure; ecdf(lat); grid on; xlabel('Latency [s]'); ylabel('CDF'); title('Bundle Delivery Latency CDF');
    figure; histogram(lat,30); xlabel('Latency [s]'); ylabel('Count'); title('Latency histogram');
end

% throughput plots
figure('Name','Throughput (kB/s)'); plot(t/60, tx_pr_log/1024, 'b', t/60, tx_relay_log/1024, 'r'); grid on;
xlabel('Time [min]'); ylabel('kB/s'); legend('Primary->Relay','Relay->GS');

% buffer backlog
figure('Name','Buffer Backlog (MB)'); plot(t/60, primary_buf_log/1e6,'b','LineWidth',1.2); hold on;
plot(t/60, relay_buf_log/1e6,'r','LineWidth',1.2); grid on;
xlabel('Time [min]'); ylabel('MB'); legend('Primary','Relay'); title('Buffer Backlog');

% SNR timeline
figure('Name','SNR (dB)'); plot(t/60, pr_snr_db, 'b', t/60, rg_snr_db, 'r'); grid on;
xlabel('Time [min]'); ylabel('SNR [dB]'); legend('Pr->Relay','Relay->GS'); title('Link SNR');

% delivered count/time
% Delivered latency
if isempty(delivered_bundles)
    disp('No bundles delivered. Try increasing sim time or improving links.');
else
    lat = [delivered_bundles.deliverTime] - [delivered_bundles.genTime];
    fprintf('Delivered bundles: %d\n', length(delivered_bundles));
    fprintf('Median latency: %.1f s, 90th percentile: %.1f s\n', median(lat), prctile(lat,90));
    
    % Manual CDF plot (no toolbox needed)
    figure('Name','Bundle Delivery Latency CDF');
    lat_sorted = sort(lat);
    cdf_vals = (1:length(lat_sorted)) / length(lat_sorted);
    plot(lat_sorted, cdf_vals, 'LineWidth', 1.5);
    grid on; xlabel('Latency [s]'); ylabel('CDF'); title('Bundle Delivery Latency CDF');
    
    % Histogram
    figure('Name','Latency Histogram');
    histogram(lat, 30, 'FaceColor', [0.2 0.4 0.8]); 
    xlabel('Latency [s]'); ylabel('Count'); title('Latency histogram');
end


%% -------------------- NOTES -------------------------------------------
% - This is a mission-realistic emulator: to be fully ground-truth accurate you should
%   replace sinusoidal distance models with orbital ephemerides (GMAT/SGP4) and compute
%   line-of-sight and access windows exactly.
% - The BER-to-fragment loss mapping is approximated. Real modcods and FEC change packet
%   success probabilities significantly; use modem lookup tables for higher fidelity.
% - CFDP reliability requires ACKs and retransmits; here we emulate per-fragment retransmit
%   with a cap on attempts. For full CFDP, include metadata ACKing, block checks, and state.
% - Tune Pt, antenna gains, bandwidth, and codingEfficiency to match realistic hardware.

