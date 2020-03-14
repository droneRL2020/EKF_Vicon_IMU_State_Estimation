position = [p_x;p_y;p_z];
orientation = [q_x;q_y;q_z];
velocity = [v_x;v_y;v_z];
bias_gyro = [bg_x;bg_y;bg_z];
bias_acc = [ba_x;ba_y;ba_z];
---------------------------------------------------------------------
r11 = cos(q_z)*cos(q_y) - sin(q_x)*sin(q_z)*sin(q_y);
r12 = -cos(q_x)*cos(q_z);
r13 = cos(q_z)*sin(q_y) + cos(q_y)*sin(q_x)*sin(q_z);
r21 = cos(q_y)*sin(q_z) + cos(q_z)*sin(q_x)*sin(q_y);
r22 = cos(q_x)*cos(q_z);
r33 = sin(q_z)*sin(q_y) - cos(q_z)*cos(q_y)*sin(q_x);
r31 = -cos(q_x)*sin(q_y);
r32 = sin(q_x);
r33 = cos(q_x)*cos(q_y);

R=[r11 r12 r13;r21 r22 r23;r31 r32 r33]
---------------------------------------------------------------------
G=[cos(q_y) 0 -cos(q_x)*sin(q_y);0 1 sin(q_x);sin(q_y) 0 cos(q_x)*cos(q_y)]
---------------------------------------------------------------------
q_dot=inv(G)*[wm_x-bg_x-ng_x;wm_y-bg_y-ng_y;wm_z-bg_z-ng_z];
q_dotx=q_dot(1);
q_doty=q_dot(2);
q_dotz=q_dot(3);
----------------------------------------------------------------------
p_dot_dot=[0;0;-9.81]+R*[a_x-ba_x-na_x;a_y-ba_y-na_y;a_z-na_z-na_z];
p_dot_dotx=p_dot_dot(1);
p_dot_doty=p_dot_dot(2);
p_dot_dotz=p_dot_dot(3);
-----------------------------------------------------------------------
x=[positin;orientation;velocity;bias_gyro;bias_acc];
x_dot=[velocity;q_dot;p_dot_dot;nbg_x;nbg_y;nbg_z;nba_x;nba_y;mbaz];
n=[ng_x;ng_y;ng_z;na_x;na_y;na_z;nbg_x;nbg_y;nbg_z;nba_x;nba_y;mbaz];
-----------------------------------------------------------------------
A=jacobian(x_dot,x);
U=jacobian(x_dot,n);

Q = (1e-3)*eye(12);
------------------------------------------------------------------------
C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0 0 0 0];

R = (1e-3)*eye(6)
------------------------------------------------------------------------

uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1));
covarPrev = eye(15);



