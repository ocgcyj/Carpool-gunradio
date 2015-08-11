function [NewEvents] = action(event)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Global Parameter
global MTU;
global RTS_size CTS_size ACK_size;
global CCA_time SLOT_time SIFS_time DIFS_time RTS_time CTS_time ACK_time PLCP_time;
global CW_min CW_max;
global RATE_List AP_List CLIENT_List SEND_List RECEIVE_List Event_List;
global Node Pkt_Buff;
% Debug related
global debug AckCount_List collisionCount CrackCount_List crackCount SentPktCount_List;
global AggregationByteCount  AggregationUserCount AggregationPktNum_Ack_Count Count;
global Buff_len_List;
global CW_List;
global PDR_List CRC_List;
% others
global busy collision;
global DATA_Rate Basic_Rate;
% Traffic related
global Traffic_interval;
global Traffic_delay_List Client_Traffic_interval;
% Aggregation related
global Aggregation_delay Aggregation_ra_num Aggregation_byte Pkt_Buff_len Aggregation_ra_index_List Aggregation_ra_List Ack_List;
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
        
    case 'nav_timeout' 
        t = event.time; 
        i = event.node;
        % swithch i to idle mode
        if strcmp(Node(i).mode,'rx')
            Node(i).mode = 'idle'; 
        end 
        % all pkts are acked, remove all acked pkts from Pkt_Buff
        if length(Aggregation_ra_List{i}) == length(Ack_List{i})
            Pkt_Buff{i}([Aggregation_ra_index_List{i}(:).index]) = [];
            % reset the backoff counter
            Node(i).CW = CW_min;
            
            if debug
               AggregationPktNum_Ack_Count{i} = [AggregationPktNum_Ack_Count{i};[length([Aggregation_ra_index_List{i}(:).index]) length([Aggregation_ra_index_List{i}(:).index]) length(Pkt_Buff{i})]];
            end
            
        % collision or some pkts are cracked, selectively remove the acked pkts from Pkt_Buff 
        else
            [~,index1,~]=intersect(Aggregation_ra_List{i},Ack_List{i},'stable');
            Pkt_Buff{i}([Aggregation_ra_index_List{i}(index1).index]) = [];
            % double the backoff counter when pkt is cracked or collision
            Node(i).CW = min(Node(i).CW + 1,CW_max);
                       
            if debug
               AggregationPktNum_Ack_Count{i} = [AggregationPktNum_Ack_Count{i};[length([Aggregation_ra_index_List{i}(:).index]) length([Aggregation_ra_index_List{i}(index1).index]) length(Pkt_Buff{i})]];
            end
            
        end               
        % release the channel immediately after ack is received 
        newevent.time = t;
        newevent.type = 'update_channel_idle';
        newevent.node = i;
        newevent.pkt = [];
        NewEvents = [NewEvents;newevent]; clear newevent;
        % more pkt are waiting to be sent
        if ~isempty(Pkt_Buff{i}) && strcmp(Node(i).mode,'idle')
            newevent.time = t;
            newevent.type = 'start_to_send';
            newevent.node = i;
            newevent.pkt = [];% add RTS here if enable
            NewEvents = [NewEvents;newevent]; clear newevent;
        end
                
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
        if i ==1 || i == 2
            % AP
            Aggregation_byte = inf;
        else
            % Client
            Aggregation_byte = 200*8;
        end
        
        % clear Aggregation_ra_index_List
        Aggregation_ra_index_List{i} = [];
        % 1. Scan the Pkt_Buff within the Pkt_Buff_len, and return the unique ra of pkt and corresponding index in the same order as Pkt_Buff
        Aggregation_ra_List{i} = unique([Pkt_Buff{i}(1:min(Pkt_Buff_len,length(Pkt_Buff{i}))).ra],'stable');
        % 2. return the ra of pkt and corresponding index whose length upper-bound is limited to Aggregation_ra_num
        Aggregation_ra_List{i} = Aggregation_ra_List{i}(1:min(Aggregation_ra_num,length(Aggregation_ra_List{i})));
        % 3. construct Aggregation_ra_index_List
        for k = 1:length(Aggregation_ra_List{i})
            Aggregation_ra_index_List{i}(k).ra = Aggregation_ra_List{i}(k);
            Aggregation_ra_index_List{i}(k).index = find([Pkt_Buff{i}(1:min(Pkt_Buff_len,length(Pkt_Buff{i}))).ra] == Aggregation_ra_List{i}(k));
        end
        index1 = sort([Aggregation_ra_index_List{i}(:).index]); % store the top Aggregation_ra_num ra index
        % If the total Aggregation_ra_index_List byte from Pkt_Buff < Aggregation_byte,
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
        
        % clear Aggregation_ra_index_List
        Aggregation_ra_index_List{i} = [];
        
        % 4. reconstruct the Aggregation_ra_index_List such that total Aggregation_ra_index_List byte from Pkt_Buff < Aggregation_byte
        index2 = [];
        for k = 1:length(index1)
            if sum([Pkt_Buff{i}(index1(1:k)).len]) <= Aggregation_byte
                index2 = [index2,index1(k)];      
            end
        end
        index3 = [Pkt_Buff{i}(index1).ra]; % store the ra list based on index1
        index3 = index3(1:length(index2));
        Aggregation_ra_List{i} = unique(index3,'stable');
        for k = 1:length(Aggregation_ra_List{i})
            Aggregation_ra_index_List{i}(k).ra = Aggregation_ra_List{i}(k);
            Aggregation_ra_index_List{i}(k).index = index2(index3 ==Aggregation_ra_List{i}(k));
        end
        
        newevent.time = t + CCA_time;
        newevent.type = 'wait_for_channel';
        newevent.node = i;
        newevent.pkt.ta = i;
        newevent.pkt.ra = Aggregation_ra_List{i};
        % 5. aggregate the pkt 
        newevent.pkt.len = sum([Pkt_Buff{i}(index2).len]);
        newevent.pkt.type = 'data';
        newevent.pkt.time = t;
        NewEvents = [NewEvents;newevent]; clear newevent;
        clear index1; clear index2;clear index3;
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
                % clear RECEIVE_List before trasmission each time
                RECEIVE_List{i} = [];
                % clear Ack_List before trasmission each time
                Ack_List{i} = [];
                % swithch j(Aggregation_ra_List) to rx mode and set 'data' event for each ra
                for k = 1:length(j)
                    if strcmp(Node(j(k)).mode,'idle')
                        Node(j(k)).mode = 'rx'; 
                    end
                    % uplink traffic
                    if i == 1 || i == 2
                        newevent.time = t + PLCP_time + event.pkt.len/DATA_Rate;
                    else
                        newevent.time = t + PLCP_time + event.pkt.len/Basic_Rate;
                    end
                    newevent.type = 'receive';
                    newevent.node = i;
                    newevent.pkt.ta = i;
                    newevent.pkt.ra = j(k);
                    newevent.pkt.len = event.pkt.len;
                    newevent.pkt.type = 'data';
                    newevent.pkt.time = t;
                    NewEvents = [NewEvents;newevent]; clear newevent;
                end
                % set a nav_timeout event
                
                % uplink traffic
                if i == 1 || i == 2
                    newevent.time = t + PLCP_time + event.pkt.len/DATA_Rate + length(j) * (SIFS_time + PLCP_time + ACK_time) + 0.1*SLOT_time;
                else
                    newevent.time = t + PLCP_time + event.pkt.len/Basic_Rate + length(j) * (SIFS_time + PLCP_time + ACK_time) + 0.1*SLOT_time;
                end
                
                newevent.type = 'nav_timeout';
                newevent.node = i;
                newevent.pkt = [];
                NewEvents = [NewEvents;newevent]; clear newevent;
                if debug
                    AggregationUserCount{i} =[AggregationUserCount{i};length(Aggregation_ra_List{i})];
                    AggregationByteCount{i} =[AggregationByteCount{i};event.pkt.len/8];
                    Count{end+1} = [Aggregation_ra_index_List{i}(:).index];
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
                % CRCCheck = 1;
                
                % Get the Num of Block Pkt of dedicated RA
                BlockPktNum = length( [Aggregation_ra_index_List{i}( Aggregation_ra_List{i}==j ).index] );
                % Get the Num of Block Pkt of before dedicated RA
                PreBlockPktNum = length( [Aggregation_ra_index_List{i}( 1:( find(Aggregation_ra_List{i}==j)-1 ) ).index] );
                %Get the Pkt Length for one Block
                indexTemp = [Aggregation_ra_index_List{i}( Aggregation_ra_List{i}==j ).index];
                PktLen = Pkt_Buff{i}( indexTemp(1) ).len;
                % set a Packet Delivery Rate based on BaselinePer or
                % EnhancementPer
                
                if i == 1 || i == 2
                   PDR = EnhancementPdr( (PreBlockPktNum + 1 + BlockPktNum)/2*PktLen, BlockPktNum*PktLen);
                else
                   PDR = BaselinePdr( (PreBlockPktNum + 1 + BlockPktNum)/2*PktLen, BlockPktNum*PktLen);
                end
                CRCCheck =  randsrc(1,1,[1 0; PDR 1-PDR]);
               
                RECEIVE_List{i} = [RECEIVE_List{i},j];
                % no collision, only one sender exists
                if length(SEND_List) == 1 && collision == 0
                    % remove the sending node from SEND_List
                    if length(RECEIVE_List{i}) == length(Aggregation_ra_List{i})
                        SEND_List(SEND_List == i) = [];
                    end
                    
                    if debug
                        PDR_List{i} = [PDR_List{i};PDR];
                        CRC_List{i} = [CRC_List{i};CRCCheck];
                    end
                
                    if CRCCheck 
                        % return ack
                        newevent.time = t + SIFS_time + (find(Aggregation_ra_List{i}==j) - 1) * (PLCP_time + ACK_time + SIFS_time);
                        newevent.type = 'send';
                        newevent.node = j;
                        newevent.pkt.ta = j;
                        newevent.pkt.ra = i;
                        newevent.pkt.len = ACK_size/8;
                        newevent.pkt.type = 'ack';
                        newevent.pkt.time = t;
                        NewEvents = [NewEvents;newevent]; clear newevent;
                        if debug
                            index2 =  [Aggregation_ra_index_List{i}(Aggregation_ra_List{i}==j).index];
                             for k = 1:length(index2)
                                 % VOIP traffic
                                if Pkt_Buff{i}(index2(k)).len == 120*8
                                    Traffic_delay_List{i} = [Traffic_delay_List{i};t - ...
                                    sum([ Pkt_Buff{i}( [Aggregation_ra_index_List{i}((find(Aggregation_ra_List{i}==j)+1:end)).index]).len])/DATA_Rate - ...    
                                    Pkt_Buff{i}(index2(k)).time];
                                end
                             end 
                            clear index2;
                        end
                    % pkt is cracked
                    else     
                        % swithch j to idle mode
                        if strcmp(Node(j).mode,'rx')
                            Node(j).mode = 'idle'; 
                        end 
                        if debug
                            crackCount = crackCount + 1;
                            CrackCount_List{i} = CrackCount_List{i} + 1;
                        end
                    end
                % collision happens, several senders exists    
                else
                    % remove the collision node which will be handled 
                    if length(RECEIVE_List{i}) == length(Aggregation_ra_List{i})
                        SEND_List(SEND_List == i) = [];
                    end
                    if isempty(SEND_List)
                      % reset collison flag by last collison node which will be handled
                      collision = 0; 
                      if debug, collisionCount = collisionCount + 1; end
                    else
                      collision = 1;  
                    end
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
                % keep j to idle mode
                Node(j).mode = 'rx';
                
                if debug
                   AckCount_List{j} = AckCount_List{j} + 1; 
                   SentPktCount_List{j} = SentPktCount_List{j} + sum([Pkt_Buff{j}(Aggregation_ra_index_List{j}(Aggregation_ra_List{j}==i).index).len]); 
                end
                
                % add the node belonging to this ack to Ack_List
                Ack_List{j} = [Ack_List{j},i];
                
        end
        
    case 'update_channel_busy'
        busy = 1;
    case 'update_channel_idle'
        busy = 0;
        
end
        
return