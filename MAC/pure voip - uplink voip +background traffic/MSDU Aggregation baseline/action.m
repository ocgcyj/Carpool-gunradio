function [NewEvents] = action(event)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Global Parameter
global MTU;
global RTS_size CTS_size ACK_size;
global CCA_time SLOT_time SIFS_time DIFS_time RTS_time CTS_time ACK_time PLCP_time;
global CW_min CW_max;
global RATE_List AP_List CLIENT_List SEND_List Event_List;
global Node Pkt_Buff;
% Debug related
global debug AckCount_List CrackCount_List collisionCount  AggregationPktNum_Ack_Count crackCount SentPktCount_List;
global AggregationByteCount Count;
global Buff_len_List;
global CW_List;
global PDR_List CRC_List;
% others
global busy collision;
global DATA_Rate Basic_Rate;
% Traffic related
global Traffic_interval Client_Traffic_interval;
global Traffic_delay_List;
% Aggregation related
global Aggregation_delay Aggregation_byte Pkt_Buff_len Aggregation_index_List;
% USRP PHY Trace
global Baseline Enhancement;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function
NewEvents = [];
switch event.type
    case 'generate_traffic'
        t = event.time; 
        i = event.node;
        % push new pkt into Pkt_Buff
        Pkt_Buff{i} = [Pkt_Buff{i};event.pkt];
        % uplink traffic
        if i == 1 || i == 2
            % AP
            newevent.time = t +  Traffic_interval;
            newevent.pkt = generate_traffic(i,newevent.time,120);
        else
            % Client
            newevent.time = t + Client_Traffic_interval;
            newevent.pkt = generate_traffic(i,newevent.time,200);
        end
        newevent.type = 'generate_traffic';
        newevent.node = i;
        NewEvents = [NewEvents;newevent]; clear newevent;
        if ~isempty(Pkt_Buff{i}) && strcmp(Node(i).mode,'idle')
            newevent.time = t;
            newevent.type = 'start_to_send';
            newevent.node = i;
            newevent.pkt = [];% add RTS here if enable
            NewEvents = [NewEvents;newevent]; clear newevent;
        end
        
    case 'retransmit' 
        t = event.time; 
        i = event.node;
        % timeout, no ack,(collision or pkt cracked) switch rx mode to idle
        % swithch i to idle mode
        if strcmp(Node(i).mode,'rx')
            Node(i).mode = 'idle'; 
        end 
        % double the backoff counter when pkt is cracked or collision
        Node(i).CW = min(Node(i).CW + 1,CW_max);
        newevent.time = t;
        newevent.type = 'start_to_send';
        newevent.node = i;
        newevent.pkt = [];% add RTS here if enable
        NewEvents = [NewEvents;newevent]; clear newevent;
        
    case 'start_to_send'
        t = event.time; 
        i = event.node;
        % indicate that the node is sending pkt
        if strcmp(Node(i).mode,'tx')
           return; 
        end
        % indicate that the node is receving pkt
        if strcmp(Node(i).mode,'rx')
           return; 
        end
        % check the Event_List, to avoid multiple 'wait_for_channel' events
        % overlapping
        index = find([Event_List(:,1).node] == i);
        if ~isempty(index)
            for k = 1:length(index)
                if strcmp(Event_List(index(k),1).type, 'wait_for_channel') ...
                || strcmp(Event_List(index(k),1).type, 'start_to_backoff') ...
                || strcmp(Event_List(index(k),1).type, 'backoff') ...
                || strcmp(Event_List(index(k),1).type, 'aggregation')
                    return;
                end
            end
        end
        
        newevent.time = t;
        newevent.type = 'aggregation';
        newevent.node = i;
        newevent.pkt = [];
        NewEvents = [NewEvents;newevent]; clear newevent;
     
    case 'aggregation'    
        t = event.time; 
        i = event.node;
        % indicate that the node is sending pkt
        if strcmp(Node(i).mode,'tx')
           return; 
        end
        % indicate that the node is receving pkt
        if strcmp(Node(i).mode,'rx')
           return; 
        end
        
        % uplink traffic
        if i == 1 || i == 2
            % AP
            Aggregation_byte = inf;
        else
            % Client
            Aggregation_byte = 200*8;
        end
        
        % 1. Scan the Pkt_Buff within the Pkt_Buff_len, and return the index of Pkts whose ra is equal to the 1st Pkt 's ra
        index1 = find([Pkt_Buff{i}(1:min(Pkt_Buff_len,length(Pkt_Buff{i}))).ra] == Pkt_Buff{i}(1).ra);
        index2 = [];
        % If the total 1st Pkt 's byte from Pkt_Buff < Aggregation_byte,
        % check again after a while
        if sum([Pkt_Buff{i}(index1).len]) < Aggregation_byte && ... 
           (t - Pkt_Buff{i}(1).time) < Aggregation_delay
            newevent.time = t + Traffic_interval;
            newevent.type = 'aggregation';
            newevent.node = i;
            newevent.pkt = [];
            NewEvents = [NewEvents;newevent]; clear newevent; 
            return
        end
        % 2. return the index of Pkts whose total len <= Aggregation_len
        for k =1:length(index1) 
            if sum([Pkt_Buff{i}(index1(1:k)).len]) <= Aggregation_byte
                index2 = [index2,index1(k)];      
            end
        end
        Aggregation_index_List{i} = index2;
        clear index1; clear index2;
        
        newevent.time = t + CCA_time;
        newevent.type = 'wait_for_channel';
        newevent.node = i;
        newevent.pkt.ta = i;
        newevent.pkt.ra = Pkt_Buff{i}(1).ra;
        % 3. aggregate the pkt 
        newevent.pkt.len = sum([Pkt_Buff{i}(Aggregation_index_List{i}).len]);
        newevent.pkt.type = 'data';
        newevent.pkt.time = t;
        NewEvents = [NewEvents;newevent]; clear newevent;
        
        if debug
            Buff_len_List{i} = [Buff_len_List{i};length(Pkt_Buff{i})];
        end
        
    case 'wait_for_channel'
        t = event.time; 
        i = event.node;
        % indicate that the node is sending pkt
        if strcmp(Node(i).mode,'tx')
           return; 
        end
        % indicate that the channel is idle, wait for DIFS
        if ~busy && strcmp(Node(i).mode,'idle')
            newevent.time = t + DIFS_time;
            newevent.type = 'start_to_backoff';
            newevent.node= i;
            newevent.pkt = event.pkt;
            NewEvents = [NewEvents;newevent]; clear newevent;
        % indicate that the channel is busy, reveiving by itself or sending by others 
        else 
            newevent.time = t + CCA_time;
            newevent.type = 'wait_for_channel';
            newevent.node = i;
            newevent.pkt = event.pkt;
            NewEvents = [NewEvents;newevent]; clear newevent;
        end
        
    case 'start_to_backoff'
        t = event.time; 
        i = event.node;
        % indicate that the node is sending pkt
        if strcmp(Node(i).mode,'tx')
           return; 
        end 
        % indicate that the channel is idle
        if ~busy && strcmp(Node(i).mode,'idle')
            % randomly draw the backoff number
            if isnan(Node(i).counter) 
                Node(i).counter = randi(2^Node(i).CW - 1);
                if debug
                    CW_List{i} = [CW_List{i};Node(i).counter];
                end
            end
            
            % 0 indicates that this node counts down to zero last time and
            % needs to send immediately and no need to backoff again
            if Node(i).counter == 0
                newevent.time = t;
                newevent.type = 'backoff';
                newevent.node = i;
                newevent.pkt = event.pkt;
                NewEvents = [NewEvents;newevent]; clear newevent;
            % start to backoff
            else
                newevent.time = t + SLOT_time;
                newevent.type = 'backoff';
                newevent.node = i;
                newevent.pkt = event.pkt;
                NewEvents = [NewEvents;newevent]; clear newevent;
            end
        % channel becomes busy during DIFS, wait until the channel is free    
        else
            newevent.time = t + CCA_time;
            newevent.type = 'wait_for_channel';
            newevent.node = i;
            newevent.pkt = event.pkt;
            NewEvents = [NewEvents;newevent]; clear newevent;
        end
            
    case 'backoff'
        t = event.time; 
        i = event.node;
        % indicate that the node is sending pkt
        if strcmp(Node(i).mode,'tx')
           return; 
        end 
        if Node(i).counter > 0
            Node(i).counter = Node(i).counter - 1;
        end
        % the channel is idle and the node is still idle
        if ~busy && strcmp(Node(i).mode,'idle')
            % ready to send the packet
            if Node(i).counter == 0
                % nan indicates that this node succeed to contend this channel
                Node(i).counter = nan;
                newevent.time = t;
                newevent.type = 'send';
                newevent.node = i;
                newevent.pkt = event.pkt;
                NewEvents = [NewEvents;newevent]; clear newevent;   
            % continue backoff
            else
                newevent.time = t + SLOT_time;
                newevent.type = 'backoff';
                newevent.node = i;
                newevent.pkt = event.pkt;
                NewEvents = [NewEvents;newevent]; clear newevent;  
            end
        % channel becomes busy during backoff count-down
        else
            newevent.time = t + CCA_time;
            newevent.type = 'wait_for_channel';
            newevent.node = i;
            newevent.pkt = event.pkt;
            NewEvents = [NewEvents;newevent]; clear newevent;
        end
        
    case 'send'
        % transmission from i to j
        t = event.time; 
        i = event.node;
        j = event.pkt.ra;
        switch event.pkt.type
            case 'data'
                % swithch i to tx mode
                if strcmp(Node(i).mode,'idle')
                   Node(i).mode = 'tx'; 
                end 
                % swithch j to rx mode
                if strcmp(Node(j).mode,'idle')
                    Node(j).mode = 'rx'; 
                end
                % update the channel after a while by 1st sending node
                if isempty(SEND_List)
                    newevent.time = t + 0.5*SLOT_time;
                    newevent.type = 'update_channel_busy';
                    newevent.node = i;
                    newevent.pkt = [];
                    NewEvents = [NewEvents;newevent]; clear newevent;
                end
                % add to SEND_List for collision dection
                SEND_List = [SEND_List;i];
                % uplink traffic
                if i == 1 || i ==2
                    newevent.time = t + PLCP_time + event.pkt.len/DATA_Rate;
                else
                    newevent.time = t + PLCP_time + event.pkt.len/Basic_Rate;
                end
                
                newevent.type = 'receive';
                newevent.node = i;
                newevent.pkt = event.pkt;
                NewEvents = [NewEvents;newevent]; clear newevent;
                if debug
                    AggregationByteCount{i} =[AggregationByteCount{i};event.pkt.len/8];
                    Count{end+1} = [Aggregation_index_List{i}];
                end
            case 'ack'
                % switch i to tx mode
                if strcmp(Node(i).mode,'rx')
                    Node(i).mode = 'tx'; 
                end
                % switch j to rx mode
                if strcmp(Node(j).mode,'tx')
                   Node(j).mode = 'rx';
                end
                newevent.time = t + PLCP_time + ACK_time;
                newevent.type = 'receive';
                newevent.node = i;
                newevent.pkt.ta = i;
                newevent.pkt.ra = j;
                newevent.pkt.len = ACK_size/8;
                newevent.pkt.type = 'ack';
                newevent.pkt.time = t;
                NewEvents = [NewEvents;newevent]; clear newevent;
        end
        
    case 'receive'
        t = event.time; 
        i = event.node;
        j = event.pkt.ra;
        switch event.pkt.type
            case 'data'
                % swithch i to rx mode
                if strcmp(Node(i).mode,'tx')
                    Node(i).mode = 'rx'; 
                end 
                %CRCCheck = 1;
                
                % set a Packet Delivery Rate based on BaselinePer or
                % EnhancementPer
                PDR = BaselinePdr( event.pkt.len/2, event.pkt.len );
                CRCCheck =  randsrc(1,1,[1 0; PDR 1-PDR]);
                
                % no collision, only one sender exists
                if length(SEND_List) == 1 && collision == 0
                    % remove the sending node from SEND_List
                    SEND_List(SEND_List == i) = [];
                    
                    if debug
                        PDR_List{i} = [PDR_List{i};PDR];
                        CRC_List{i} = [CRC_List{i};CRCCheck];
                    end
                    
                    if CRCCheck 
                        % return ack
                        newevent.time = t + SIFS_time;
                        newevent.type = 'send';
                        newevent.node = j;
                        newevent.pkt.ta = j;
                        newevent.pkt.ra = i;
                        newevent.pkt.len = ACK_size/8;
                        newevent.pkt.type = 'ack';
                        newevent.pkt.time = t;
                        NewEvents = [NewEvents;newevent]; clear newevent;
                        if debug
                            for k = 1:length(Aggregation_index_List{i})
                                % VOIP traffic
                                if Pkt_Buff{i}(Aggregation_index_List{i}(k)).len == 120*8
                                    Traffic_delay_List{i} = [Traffic_delay_List{i};t - Pkt_Buff{i}(Aggregation_index_List{i}(k)).time];
                                end
                            end 
                            
                            AggregationPktNum_Ack_Count{i} = [AggregationPktNum_Ack_Count{i};[length(Aggregation_index_List{i}) length(Aggregation_index_List{i}) length(Pkt_Buff{i})]];
                        end
                    % pkt is cracked    
                    else   
                        newevent.time = t + SIFS_time + PLCP_time + ACK_time;% no ack, timeout 
                        newevent.type = 'retransmit';
                        newevent.node = i;
                        newevent.pkt = event.pkt;
                        NewEvents = [NewEvents;newevent]; clear newevent;
                        % release the channel after a while by last collision node
                        newevent.time = t + SIFS_time + PLCP_time + ACK_time;% no ack, timeout 
                        newevent.type = 'update_channel_idle';
                        newevent.node = j;
                        newevent.pkt = [];
                        NewEvents = [NewEvents;newevent]; clear newevent;
                        % swithch j to idle mode
                        if strcmp(Node(j).mode,'rx')
                            Node(j).mode = 'idle'; 
                        end 
                        if debug
                            crackCount = crackCount + 1;
                            CrackCount_List{i} = CrackCount_List{i} + 1;
                            AggregationPktNum_Ack_Count{i} = [AggregationPktNum_Ack_Count{i};[length(Aggregation_index_List{i}) 0 length(Pkt_Buff{i})]];
                        end
                    end
                % collision happens, several senders exists    
                else
                    % remove the collision node which will be handled 
                    SEND_List(SEND_List == i) = [];
                    if isempty(SEND_List)
                      % reset collison flag by last collison node which will be handled
                      collision = 0; 
                      % release the channel after a while by last collision node 
                      newevent.time = t + SIFS_time + PLCP_time + ACK_time;
                      newevent.type = 'update_channel_idle';
                      newevent.node = j;
                      newevent.pkt = [];
                      NewEvents = [NewEvents;newevent]; clear newevent;
                      if debug
                          collisionCount = collisionCount + 1; 
                          AggregationPktNum_Ack_Count{i} = [AggregationPktNum_Ack_Count{i};[length(Aggregation_index_List{i}) 0 length(Pkt_Buff{i})]];
                      end
                    else
                      collision = 1;  
                    end
                    % handle the collison pkt, retransmission
                    newevent.time = t + SIFS_time + PLCP_time + ACK_time;% no ack, timeout 
                    newevent.type = 'retransmit';
                    newevent.node = i;
                    newevent.pkt = event.pkt;
                    NewEvents = [NewEvents;newevent]; clear newevent;
                    % swithch j to idle mode
                    if strcmp(Node(j).mode,'rx')
                        Node(j).mode = 'idle'; 
                    end 
                end
                
            case 'ack'
                 % switch i to idle mode
                if strcmp(Node(i).mode,'tx')
                    Node(i).mode = 'idle'; 
                end
                % switch j to idle mode
                if strcmp(Node(j).mode,'rx')
                   Node(j).mode = 'idle';
                end
                
                if debug
                   AckCount_List{j} = AckCount_List{j} + 1; 
                   SentPktCount_List{j} = SentPktCount_List{j} + sum([Pkt_Buff{j}(Aggregation_index_List{j}).len]);
                end
                
                % remove the aggregated pkts from Pkt_Buff
                Pkt_Buff{j}(Aggregation_index_List{j}) = [];                
                % reset the backoff counter
                Node(j).CW = CW_min;
                % release the channel immediately after ack is received 
                newevent.time = t;
                newevent.type = 'update_channel_idle';
                newevent.node = j;
                newevent.pkt = [];
                NewEvents = [NewEvents;newevent]; clear newevent;
                % more pkt are waiting to be sent
                if ~isempty(Pkt_Buff{j}) && strcmp(Node(j).mode,'idle')
                    newevent.time = t;
                    newevent.type = 'start_to_send';
                    newevent.node = j;
                    newevent.pkt = [];% add RTS here if enable
                    NewEvents = [NewEvents;newevent]; clear newevent;
                end
        end
    case 'update_channel_busy'
        busy = 1;
    case 'update_channel_idle'
        busy = 0;
        
end
        
return