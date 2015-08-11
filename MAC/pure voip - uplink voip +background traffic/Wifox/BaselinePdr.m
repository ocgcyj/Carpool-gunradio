function [PktPdr] = BaselinePdr(PktBitLoc,PktLen)
global Baseline

PktSymLoc = ceil(PktBitLoc/(52*6)/10);% The average symbol location of the block Pkts destined to one RA 2Mhz to 20Mhz
%PktSymNum = PktLen/(52*6); % convert the Pkt Byte to symbol Num, 52subcarrier qam64 
PktSymNum = ceil(120*8/(52*6));
Num = randi(length(Baseline)); % randomly extract the trace data from USRP
PktBer = Baseline(Num,1)*PktSymLoc + Baseline(Num,2); % calculate the BER
PktPdr = (1-PktBer)^(PktSymNum*52); % convert BER to PDR 

return
