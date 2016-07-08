function writeVibrationalMotors(COM_port,motor_pin,frequency,interval,number,amplitude)

no_COM={'COM not detected'};
readDat=0;
duration=1/(frequency*2);

if sum(COM_port{:})~=sum(no_COM{:})

s = serial(COM_port{:});    % Create Serial Object
    set(s,'BaudRate',9600);         % Set baud rate
    fopen(s);                       % Open the port
    
    
    writeData=char(uint8([2 motor_pin 1 number duration duration amplitude]));
    fwrite(s,writeData,'uchar');
    
    fclose(s);              % Close and delete COM object
    delete(s);
end