

fileID1 = fopen('tocubecurrent.txt','r');
fileID2 = fopen('tocubedesired.txt','r');
formatSpec = '%f';
A = fscanf(fileID1,formatSpec)
numitems1 = length(A)/3
fclose(fileID1)
fileID1 = fopen('tocubecurrent.txt','r');
B = fscanf(fileID1,formatSpec,[3 Inf]);
C = fscanf(fileID2,formatSpec,[3 Inf]);


B=B'
C=C'
B=B(2:length(B),:)
C=C(2:length(C),:)

Xs1=linspace(1,numitems1-1,numitems1-1)

hold on
plot(Xs1, B(:,1), 'r')
plot(Xs1, B(:,2), 'g')
plot(Xs1, B(:,3), 'b')

plot(Xs1, C(:,1), 'm')
plot(Xs1, C(:,2), 'y')
plot(Xs1, C(:,3), 'c')


fileID3 = fopen('toshelfcurrent.txt','r');
fileID4 = fopen('toshelfdesired.txt','r');
formatSpec = '%f';
D = fscanf(fileID3,formatSpec)
numitems2 = length(D)/3
fclose(fileID3)
fileID1 = fopen('toshelfcurrent.txt','r');
E = fscanf(fileID3,formatSpec,[3 Inf]);
F = fscanf(fileID4,formatSpec,[3 Inf]);



E=E'
F=F'
E=E(2:length(E),:)
F=F(2:length(F),:)

Xs2=linspace(0,numitems2-1,numitems2-1)
Xs2=Xs2+numitems1

plot(Xs2, E(:,1), 'r')
plot(Xs2, E(:,2), 'g')
plot(Xs2, E(:,3), 'b')

plot(Xs2, F(:,1), 'm')
plot(Xs2, F(:,2), 'y')
plot(Xs2, F(:,3), 'c')