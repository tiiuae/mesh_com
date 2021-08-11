clc;
csi_HT20_len=108;
no_of_subcarrier=54
%load filtered CSI dump
%<DEADBEEF><CFR_META_DATA><CFR_DATA{I_chain0/Q_chain0..}><BEEFDEAD>
fid = fopen('ath10k_cfr.txt', 'r');
cfr_word = textscan(fid, '%s')
cfr_word = cfr_word{1};
cfr_word_arr_chain0 = cfr_word(1:csi_HT20_len);
cfr_word_arr_chain1 = cfr_word(csi_HT20_len+1:csi_HT20_len+csi_HT20_len);

%chain 0 data
img_chain0 = {0}
real_chain0 = {0}
chain0_amp = {0}
chain0_phase = {0}
%chain 1 data
img_chain1 = {0}
real_chain1 = {0}
chain1_amp = {0}
chain1_phase = {0}

i = 1
for n = 1 : no_of_subcarrier
  img_chain0(n) = cfr_word_arr_chain0(i);
  img_chain1(n) = cfr_word_arr_chain1(i);
  i += 1;
  real_chain0(n) = cfr_word_arr_chain0(i);
  real_chain1(n) = cfr_word_arr_chain1(i);
  i += 1;
  img_chain0_t = cell2mat(img_chain0(n));
  img_chain1_t = cell2mat(img_chain1(n));
  real_chain0_t = cell2mat(real_chain0(n));
  real_chain1_t = cell2mat(real_chain1(n));
  val_chain0 = img_chain0_t(:,1)*img_chain0_t(:,1) + real_chain0_t(:,1)*real_chain0_t(:,1);
  val_chain1 = img_chain1_t(:,1)*img_chain1_t(:,1) + real_chain1_t(:,1)*real_chain1_t(:,1);
  %calculate amplitude sqrt(I^2 + Q^2)
  chain0_amp(n) = sqrt(val_chain0);
  chain1_amp(n) = sqrt(val_chain1);
  phase_chain0_t = img_chain0_t/real_chain0_t;
  phase_chain1_t = img_chain1_t/real_chain1_t;
  %calculate phse atan(Q/I)
  chain0_phase(n) = atan(phase_chain0_t(:,1))
  chain1_phase(n) = atan(phase_chain1_t(:,1))
end

fclose(fid);

%plot chain0 amplitude/phase
subplot(4,1,1);
plot(cell2mat(chain0_amp));
xlabel("subcarrier index");
ylabel("chain0 amplitude");

subplot(4,1,2);
plot(cell2mat(chain0_phase));
xlabel("subcarrier index");
ylabel("chain0 phase");


%plot chain1 amplitude/phase
subplot(4,1,1);
subplot(4,1,3);
plot(cell2mat(chain1_amp));
xlabel("subcarrier index");
ylabel("chain1 amplitude");

subplot(4,1,4);
plot(cell2mat(chain1_phase));
xlabel("subcarrier index");
ylabel("chain1 phase");
