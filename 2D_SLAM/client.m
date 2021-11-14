clear
i=1;
t_client=tcpip('192.168.43.19',9000,'NetworkRole','client');
t_client.InputBuffersize=100000;
fopen(t_client);
while(1)
    if  t_client.BytesAvailable>0
        data_recv=str2num(char(fread(t_client,get(t_client,'BytesAvailable'),'char')'));%从缓冲区读取数字数据
%         pause(0.05)
        H=size(data_recv);
        M(i:i+H(1)-1,:)=data_recv;
        i=i+H(1);
        figure(1)
        plot(M(:,1),M(:,2),'r-');
%         plot(data_recv(:,1),data_recv(:,2),'k-');
        hold on
        drawnow;
%         axis([-6000 1000 -3500 3500]);
%         axis([-1000 6000 -5500 500]);
        axis manual
        title("Map");
        xlabel("X/mm");
        ylabel("Y/mm");
    end
end

