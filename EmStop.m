function EmStop(Arduino,Pins,val)
    %EMERGENCY STOP
    if nargin == 2
        val = (1:5);
    end
    digitalWrite(Arduino,Pins(val,3), Pins(6,1)*ones(size(val)));
    digitalWrite(Arduino,Pins(val,2), Pins(6,1)*ones(size(val)));
    digitalWrite(Arduino,Pins(val,3), ~Pins(6,1)*ones(size(val)));
    digitalWrite(Arduino,Pins(val,2), ~Pins(6,1)*ones(size(val)));
end
