%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Global Parameter
global MTU;
global RTS_size CTS_size ACK_size;
global CCA_time SLOT_time SIFS_time DIFS_time RTS_time CTS_time ACK_time PLCP_time;
global CW_min CW_max;
global RATE_List AP_List CLIENT_List SEND_List;
global APNum ClientNum;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 802.11n PHY Parameter, HT mode,no MIMO
MTU = 1500*8;% bit
RATE_List = [6.5,65,135,195,300,480,600]*1e6; % HT mode Rate
% RATE_List = [6,9,12,18,24,36,48,54]*1e6; % Lagacy mode Rate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 802.11n MAC Parameter, HT mode,no MIMO %Max_mac_body_size = 2312*8; %MAC_header_size = (2+2+6+6+6+2+6+4)*8;  % FC+duration+a1+a2+a3+sequence+a4+fcs
RTS_size = (2+2+6+6+4)*8;   % FC+duration+ra+ta+fcs
CTS_size = (2+2+6+4)*8;     % FC+duration+ra+fcs
ACK_size = CTS_size;        % FC+duration+ra+fcs

CCA_time = 4*1e-6;
SLOT_time = 9*1e-6;
SIFS_time = 10*1e-6;
DIFS_time = 2*SLOT_time + SIFS_time;
RTS_time = RTS_size/RATE_List(1);
CTS_time = CTS_size/RATE_List(1);
ACK_time = ACK_size/RATE_List(1);
PLCP_time = 28*1e-6;% 8us(HT-STS) + 8us(HT-LTS) + 8us(HT-SIG) + 4us(HT-LTSs)% HT mode, no MIMO
% PLCP_time = 20*1e-6;% 8us(L-STS) + 8us(L-LTS) + 4us(L-SIG)% Lagacy mode

CW_min = 5;                 % 31 = 2^5-1
CW_max = 10;                % 1023 = 2^10-1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Other Parameter
AP_List = (1:58);
CLIENT_List = (59:1682);
SEND_List = [];
APNum = length(AP_List);
ClientNum = length(CLIENT_List);