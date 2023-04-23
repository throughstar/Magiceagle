function para = bird_sys_para()
    %controller
    %parameters
    para.Cf = 1;
    para.m = 1;
    para.V = 1;
    para.beta_r= 3;
    para.g = 9.8;
    para.beta_1 = 1;
    %initial
    para.z0 = [-4;1;1];
    para.theta0 = [0.4;3;16;-0.4];
end
