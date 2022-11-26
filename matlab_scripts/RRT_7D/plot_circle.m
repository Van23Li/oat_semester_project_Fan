function plot_circle(centers, radiuses, xLim, yLim, ax_h)
circ_mask = zeros(201, 201);

for i = 1:size(centers, 1)
    [x, y] = meshgrid(xLim(1) : 0.1 : xLim(2), yLim(1) : 0.1 : yLim(2));
    circle = (x - centers(i,1)).^2 + (y - centers(i,2)).^2;
    circ_mask(find(circle <= radiuses(i)^2)) = 1;
end

cmap = [1 1 1; 0 0 0];
colormap(cmap);
hold on;
im = imagesc(ax_h, xLim, yLim, circ_mask);
im.AlphaData = 0.4;
hold off

end