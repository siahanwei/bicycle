function [a,b,c,lambda,h,m,g,v]=get_bike_param(u)

    if strcmp(u,'lego')
        a=0.06;   %x distance to CG
        b=0.225;   %wheel base
        c=0.011;   %trail
        lambda=67; %steering axis angle from horizontal degrees
        h=0.08;   %height of CG
        m=0.678;      %Total mass of bicycle
        g=9.81;    %gravitational constant
        v=0.6; %Speed  in m/s 

    elseif strcmp(u,'real')
        a=0.4;   %x distance to CG
        b=1.1;   %wheel base
        c=0.08;   %trail
        lambda=70; %steering axis angle from horizontal degrees
        h=0.7;   %height of CG
        m=52.5;      %Total mass of bicycle
        g=9.81;    %gravitational constant
        v=5; %Speed  in m/s 
        
    else  
        disp('ERROR: enter either "real" or "lego" for parameters of real bike or lego bike')
    end    
    
end