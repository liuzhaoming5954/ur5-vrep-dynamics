function  model = UR5( URE, URr,inertial )
nb = 6;
bf = 1;

model.NB = nb;

for i = 1:nb
    model.jtype{i} = 'R';
    model.parent(i) = floor((i-2+ceil(bf))/bf);
    model.Xtree{i} = plux(URE{i},URr{i});
    %len(i) = l(i);
    mass = inertial.mass(i);
    CoM = inertial.CoM{i};
    Icm = inertial.I{i};
    model.I{i} = mcI( mass, CoM, Icm );
end


% drawing instructions
% l(1) = 0;
% l(2) = 0.425;
% l(3) = 0.39225;
% l(4) = 0;
% l(5) = 0;
% l(6) = 0.0823;
% taper = 1;
% 
% model.appearance.base = ...
%   { 'box', [-0.2 -0.3 -0.2; 0.2 0.3 -0.06] };
% 
% p0 = -1;
% for i = 1:nb
%   p1 = model.parent(i);
%   tap = taper;
%   if p1 == 0
%     ptap = 1;
%   else
%     ptap = l(i);
%   end
%   if ( p1 > p0 )
%     model.appearance.body{i} = ...
%       { 'cyl', [0 0 0; 1 0 0]*ptap, 0.05, ...
%         'cyl', [0 0 -0.07; 0 0 0.07], 0.08 };
%     p0 = p1;
%   else
%     model.appearance.body{i} = ...
%       { 'cyl', [0 0 0; 1 0 0]*tap, 0.05*tap };
%   end
% end


% drawing instructions
% taper = 1;
% model.appearance.base = ...
%   { 'box', [-0.2 -0.3 -0.2; 0.2 0.3 -0.06] };
% 
% p0 = -1;
% for i = 1:nb
%   p1 = model.parent(i);
%   tap = taper^(i-1);
%   if p1 == 0
%     ptap = 1;
%   else
%     ptap = taper^(p1-1);
%   end
%   if ( p1 > p0 )
%     model.appearance.body{i} = ...
%       { 'cyl', [0 0 0; 1 0 0]*0.2, 0.05*tap, ...
%         'cyl', [0 0 -0.07; 0 0 0.07]*ptap, 0.08*ptap };
%     p0 = p1;
%   else
%     model.appearance.body{i} = ...
%       { 'cyl', [0 0 0; 1 0 0]*tap, 0.05*tap };
%   end
% end
