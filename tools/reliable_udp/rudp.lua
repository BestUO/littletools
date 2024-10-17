local CustomRUDP = Proto("CustomRUDP", "CustomRUDP")
CustomRUDP_Type = ProtoField.uint8("CustomRUDP.type", "type", base.DEC)
CustomRUDP_UUID = ProtoField.guid("CustomRUDP.uuid", "uuid", base.GUID)
-- CustomRUDP_Payload = ProtoField.string("CustomRUDP.payload", "payload", base.STRING)

CustomRUDP.fields = {CustomRUDP_Type,CustomRUDP_UUID,CustomRUDP_Payload}

function CustomRUDP_dissector(buffer, pinfo, tree)
    pinfo.cols.protocol = CustomRUDP.name
    local subtree =  tree:add(CustomRUDP, buffer(), "CustomRUDP")
    
    subtree:add(CustomRUDP_Type,buffer(0,1))
    subtree:add(CustomRUDP_UUID, buffer(1,16))
    -- subtree:add(CustomRUDP_Payload, buffer(17))
    return true
end

function CustomRUDP.dissector(buf,pkt,root) 
    if CustomRUDP_dissector(buf,pkt,root) then
        --valid DT diagram
    else
        --data这个dissector几乎是必不可少的；当发现不是我的协议时，就应该调用data
        data_dis:call(buf,pkt,root)
    end
end

-- bind port to protocol
local udp_port = DissectorTable.get("udp.port")
udp_port:add(9987,CustomRUDP)