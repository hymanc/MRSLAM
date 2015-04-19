function [MAP,PIXDIM]=getTheMAP(THEIMAGE)

    
    switch THEIMAGE
        case 'Test1.png'
            A=imread(THEIMAGE);
            MAP=A(:,:,1)==0 & A(:,:,2)==0 & A(:,:,3)==0;
            PIXDIM=1;%1 meter/1 Pixel
        case 'Test2.bmp'
            A=imread(THEIMAGE);
            MAP=A(:,:,1)==0 & A(:,:,2)==0 & A(:,:,3)==0;
            PIXDIM=1;%1 meter/1 Pixel

        case 'Test3.png'
            A=imread(THEIMAGE);
            MAP=A(:,:,1)~=255 & A(:,:,2)~=255 & A(:,:,3)~=255;
            PIXDIM=1;%1 meter/1 Pixel
        case 'Test4.png'
            A=imread(THEIMAGE);
            MAP=A(:,:,1)~=255 & A(:,:,2)~=255 & A(:,:,3)~=255;
            PIXDIM=1;%1 meter/1 Pixel
        case 'paw-to-paw-maze-June.jpg'
            A=imread(THEIMAGE);
            MAP=A(:,:,1)<64;
            PIXDIM=1;%1 meter/1 Pixel
        case 'Small.png'
            A=imread(THEIMAGE);
            MAP=A(:,:,1)~=255 & A(:,:,2)~=255 & A(:,:,3)~=255;
            PIXDIM=1;%1 meter/1 Pixel
        case 'Test5.png'
            A=imread(THEIMAGE);
            MAP=A(:,:,1)~=255 & A(:,:,2)~=255 & A(:,:,3)~=255;
            PIXDIM=1;%1 meter/1 Pixel
        otherwise

    end



end