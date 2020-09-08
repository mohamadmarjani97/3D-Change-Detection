% This Function Match Bins of OT1 With OT2
function MatchingBin=BinMatching(OT1,points3D_1,OT2,points3D_2)
MatchingBin=inf(OT2.BinCount,1);
for i=1:OT2.BinCount
    Coor22=points3D_2(OT2.PointBins==i,:);
    XMeanCoor22=mean(Coor22(:,1));
    YMeanCoor22=mean(Coor22(:,2));
    ZMeanCoor22=mean(Coor22(:,3));
    [m22,~]=size(Coor22);
    if m22>=1
        i
        DisBin=inf(OT1.BinCount,1);
        for j=1:OT1.BinCount
            Coor11=points3D_1(OT1.PointBins==j,:);
            XMeanCoor11=mean(Coor11(:,1));
            YMeanCoor11=mean(Coor11(:,2));
            ZMeanCoor11=mean(Coor11(:,3));
            [m11,~]=size(Coor11);
            if m11>=1
                DX=XMeanCoor11-XMeanCoor22;
                DY=YMeanCoor11-YMeanCoor22;
                DZ=ZMeanCoor11-ZMeanCoor22;
                DisBin(j)=sqrt(DX^2+DY^2+DZ^2);
            end
        end
        MatchingBin(i)=find(DisBin==min(DisBin));
    end
end
