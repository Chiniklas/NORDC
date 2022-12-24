pic_num = 1;
for fai = 0:0.05:pi/4
    %----------------------------------------------------------------------
    % 这里你随便写你的代码 出图到f = figure(1);
    %----------------------------------------------------------------------
    robolinplot
    %----------------------------------------------------------------------
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map,'test2.gif','gif','Loopcount',inf,'DelayTime',0.2);
    else
        imwrite(I,map,'test2.gif','gif','WriteMode','append','DelayTime',0.2);
    end
    pic_num = pic_num + 1;
    %----------------------------------------------------------------------
end