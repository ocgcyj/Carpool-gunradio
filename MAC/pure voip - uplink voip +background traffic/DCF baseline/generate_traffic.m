function [Pkt]=generate_traffic(APid,time,byte)
global MTU;
global CLIENT_List;
global APNum ClientNum;

Pkt.ta = APid;
Pkt.ra = CLIENT_List(randi(ClientNum/APNum)+(APid-1)*(ClientNum/APNum));
Pkt.len = byte*8;
Pkt.type = 'data';
Pkt.time = time;
    
return