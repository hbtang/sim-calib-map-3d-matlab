function ProcDrawRes( map, mark, odo )
%PROCDRAWRES draw results

figure; grid on; hold on; axis equal;
vecPt3d_w_m = map.vecPt3d_w_m;
vecPs2d_w_b = map.vecPs2d_w_b;
% plot3(vecPt3d_w_m(:,1), vecPt3d_w_m(:,2), vecPt3d_w_m(:,3), ...
%     '.', 'Color','k','MarkerSize',15);
plot3(vecPt3d_w_m(:,1), vecPt3d_w_m(:,2), vecPt3d_w_m(:,3), ...
   's','MarkerEdgeColor','k', 'MarkerFaceColor','k', 'MarkerSize', 10);
plot(vecPs2d_w_b(:,1), vecPs2d_w_b(:,2),...
     '.-', 'Color','b','MarkerSize',5);
% plot(odo.x, odo.y,...
%      '.-', 'Color','g','MarkerSize',5);

for i = 1:mark.num
    idMk = mark.id(i);
    idMkOrd = find(mark.mkHash(:,1) == idMk);
    stamp = mark.stamp(i);
    pt3d_w_m = vecPt3d_w_m(idMkOrd,:).';
    pt3d_w_b = vecPs2d_w_b(stamp,:).';
    pt3d_w_b(3) = 0;    
    plot3([pt3d_w_m(1);pt3d_w_b(1)], [pt3d_w_m(2);pt3d_w_b(2)], [pt3d_w_m(3);pt3d_w_b(3)], ...
       '-', 'Color','r');
end

end

