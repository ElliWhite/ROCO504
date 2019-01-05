close all;

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

s1=serial('COM10','Baudrate',9600);
fopen(s1);
accX=0;
str='';
sen=0;
j=1;
x=0;


while(j<1000)
    
    str=fscanf(s1);
    sen=str2num(str);
    accX(j)=sen;
    x(j)=j;
    
    %bin first few values
    if(j<5)
        accX(j) = 0;
    end
    disp(j);
    
    j=j+1;

end;



xmin=0;
xmax=1000;
ylim=max(accX)+0.2;

plot(x,accX);
xlabel('Sample Number');
ylabel('Acceleration in X Axis (G)');
axis([xmin xmax -0.5 ylim]);



fclose(s1);
delete(s1);
clear s1;