
function R = circle(x, y, r, color, linestyle)
for i = 1:length(r)
    t = linspace(0, 2*pi);
    R = patch(x(:,i) + r(i)*cos(t), y(:,i)+r(i)*sin(t),'r'); hold on;
    R.FaceColor = 'none';
    R.EdgeColor = color;
    R.LineStyle = linestyle;
end
end