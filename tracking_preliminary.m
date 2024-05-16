% model initialisation 



load('twizy_scenario.mat');

refposition = data.ActorSpecifications.Waypoints;

xref = refposition(:,1);
yref = -refposition(:,2);

% defining vehicle parameters

% vehicle inital position

x_initial = xref(1);
y_initial = yref(1);

% calculating reference pose vectors, based on how far the vehicle travels,
% the pose is generated using one dimensional look up tables

distmatrix = squareform(pdist(refposition)); % distance matrix
diststeps = zeros(length(refposition)-1,1);

for i = 2:length(refposition)

    diststeps(i-1,1) = distmatrix(i,i-1);

end

totaldist = sum(diststeps); % this is the total distance travelled
distbypoint = cumsum([0;diststeps]); % distance for each way point
gradbypoint = linspace(0,totaldist,50);% linearize distance 

% now we linearise x and y vectors based on distance 

xref2 = interp1(distbypoint,xref,gradbypoint);
yref2 = interp1(distbypoint,yref,gradbypoint);

xref2s = smooth(gradbypoint,xref2); %smoothing the response data
yref2s = smooth(gradbypoint,yref2);



%plot(gradbypoint,xref2s);
%xlabel('distance')
%ylabel('x')

%plot(gradbypoint,yref2s);
%xlabel('distance')
%ylabel('x')


%calculating the theta vector(vehicle orientation)

thetaref = zeros(length(gradbypoint),1);
for i = 2:length(gradbypoint)
    thetaref(i,1) = atan2d((yref2(i)-yref2(i-1)),(xref2(i)-xref2(i-1)));

end

thetarefs = smooth(gradbypoint,thetaref); % smooth of theta
intialyaw = thetarefs(1)*(pi/180);%initial yaw angle

%plot(gradbypoint,thetarefs)
%xlabel('distance')
%ylabel('theta')

% creating direction vector 

direction = ones(length(gradbypoint),1);

%calculating curvature vector 

curvature = getCurvature(xref2s,yref2s);

%plot(gradbypoint,curvature);

%xlabel('distance')
%ylabel('çurvature')

function curvature = getCurvature(xref,yref)

%calculating gradient by the gradient of X and Y 

dx = gradient(xref);
d2x = gradient(dx);
dy = gradient(yref);
d2y = gradient(dy);
curvature = (dx.*d2y - dy.*d2x) ./(dx.^2 + dy.^2).^(3/2);
end













