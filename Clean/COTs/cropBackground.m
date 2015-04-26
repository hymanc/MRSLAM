function cropBackground(filefrom,fileto,background)

    if (nargin<3)
        background=[255 255 255];
        if (nargin<2)
            fileto=filefrom;
        end
    end

    A=imread(filefrom);
    sA=size(A);
    myInds=true(sA(1),sA(2));
    for a1=1:3
        myInds=A(:,:,a1)~=background(a1);
    end
    
    along1=sum(myInds,1);
    along2=sum(myInds,2);
    
    vect1=1:sA(1);
    vect2=1:sA(2);
    
    vect2=vect2(along1~=0);
    vect1=vect1(along2~=0);
    vect2=[vect2(1) vect2(end)];
    vect1=[vect1(1) vect1(end)];
    A=A(vect1(1):vect1(2),vect2(1):vect2(2),:);
    
    imwrite(A,fileto);
end