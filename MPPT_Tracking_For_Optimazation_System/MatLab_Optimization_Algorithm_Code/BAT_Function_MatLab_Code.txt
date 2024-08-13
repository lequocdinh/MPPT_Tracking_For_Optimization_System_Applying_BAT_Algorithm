function D = BAT(Vin,Iin)

coder.extrinsic('randi')

persistent u;
persistent i;
persistent j;
persistent k;                  % Iterations
persistent DP;                 % DutyCycle   
persistent DPcurrent;          % Current DutyCycle      
persistent v;                  % Velocity
persistent F;                  % Frequency
persistent Fmax;               % Maximum frequency
persistent Fmin;               % Minimum frequency
persistent A;                  % Loudness
persistent r;                  % Pulse emission rate
persistent bestDP;
persistent DPcontainer;
persistent DPfitness;
persistent P;                  % Power
persistent Pfitness;
persistent counter;            % Counter
persistent lb;                 % Lower boundary
persistent ub;                 % Upper boundary
persistent ro;                 % Initial pulse emission rate
persistent alpha;              % Constant for loudness update
persistent gamma;              % Constant for emission rate update
persistent eps;
persistent Pprev; 
persistent bool; 
persistent closeness;
persistent index;

%initialization
if (isempty(counter))
    counter=0;
    bool=0;
    index=0;
    closeness=false;
    Pprev=Vin*Iin;
    k=0;                    % Iterations
    lb=0;                   % Lower boundary
    ub=1;                   % Upper boundary
    DPcurrent=0.5;          % Current DutyCycle  
    DPcontainer=zeros(4,1);
    DPfitness=zeros(4,1);
    Fmax=0.5;               % Maximum frequency
    Fmin=0.3;               % Minimum frequency
    A=rand(4,1);            % Loudness for each DutyCycle
    r=rand(4,1);            % Pulse emission rate for each DutyCycle
    u=0;
    i=0;
    v=zeros(4,1);           % Velocities
    F=zeros(4,1);           % Frequency
    P=zeros(4,1);           % Power
    Pfitness=zeros(4,1);
    alpha=0.5;              % Constant for loudness update
    gamma=0.5;              % Constant for emission rate update
    ro=0.67;                % Initial pulse emission rate
    DP=zeros(4,1);          % Dutycycles
    %initial dc for each particle
    DP(1)=0.2;
    DP(2)=0.4;
    DP(3)=0.6;
    DP(4)=0.8;       
end

% At the first time, this if ignored (counter=0)
% Delay
if(counter>=1 && counter<300)
    D=DPcurrent;
    counter=counter+1;
    return;              % return control to the invoking function before it reaches the end of the function.
end
counter=0;               % reset the counter

% calculate the fittnes function (power) of each particle
% then compare the current value of the function with the previous one
% if the current value is better than the previous, update the value
% Note: at the first time, this if ignored (u=0)
if(u>=1 && u<=4)
    P(u)=Vin*Iin;
    DPcontainer(u)=DPcurrent;    
end

% Update if the solution improves, or not too loud
if(u>4)
    for i=1:4
        if (P(i)>=Pfitness(i)) 
            Pfitness(i)=P(i);
            DPfitness(i)=DPcontainer(i);
            if(k>=1)  
                A(i)=alpha*A(i);
                r(i)=ro*(1-exp(-gamma*k));
            end
        end
    end
end

u=u+1;
% AT the first time, this is excuted because it consist the condition...
% u==0 (initial value of u)
if(u==6)
    u=1;
end

if(u==5)
    if(bool==0)
        k=k+1;    
    end

    [Pmax,index]=max(Pfitness);            % finds the indices of the maximum values of P (max power) and returns them in output vector "index"
    bestDP=DPfitness(index);               % find the location(duty) of the particle which has max Power
    if(bool==0)
        D=bestDP;
    else
        D=DP(index);         %mean          
    end
    DPcurrent=D;
    counter=1;

    for i=1:4
        F(i)=Fmin+(Fmax-Fmin)*rand;                     %randomly chose the frequency
        v(i)=v(i)+(DPfitness(i)-bestDP)*F(i);           %update the velocity
        if DPfitness(i)>bestDP
            DP(i)=DPfitness(i)-abs(v(i));               %update the Dutycycle position
        else
            DP(i)=DPfitness(i)+abs(v(i));               %update the Dutycycle position
        end
 
        %Checking bounds/limits
        if DP(i) < lb % Check the lower limit
            DP(i) = lb;
        elseif DP(i) > ub % Check the upper limit
            DP(i) = ub;
        end

        %check the condition with r
        if rand>r(i)
            eps=-1+(1-(-1))*rand;
            DP(i)=bestDP+eps*mean(A);
        end        
    end

    if(bool==0)
        closeness=true;
        for i=1:4
            for j=1:4
                if(i~=j)
                    closeness = closeness & (abs(DP(i)-DP(j))<0.1);       %0.05
                end
            end
        end
    end

    %if the power stables already
    if(bool==1 )        
        %check if the power changes
        if(abs(Vin*Iin-Pprev)/Pprev >= 0.05)           %0.1
            
            DPcurrent=0.5;          % Current DutyCycle  
            DPcontainer=zeros(4,1);
            DPfitness=zeros(4,1);
            A=rand(4,1);            % Loudness for each DutyCycle
            r=rand(4,1);            % Pulse emission rate for each DutyCycle
            v=zeros(4,1);           % Velocities
            F=zeros(4,1);           % Frequency
            P=zeros(4,1);           % Power
            Pfitness=zeros(4,1);
            DP=zeros(4,1);          % Dutycycles
               
            DP(1)=randi([5 250])/1000;
            DP(2)=randi([250 500])/1000;
            DP(3)=randi([500 750])/1000;
            DP(4)=randi([760 995])/1000;
                
            bool=0;           
        end     
    end
        
    if(closeness==1)
        bool=1; 
        closeness=false;   
    end
    
    if(k>100)
          
            DPcurrent=0.5;          % Current DutyCycle  
            DPcontainer=zeros(4,1);
            DPfitness=zeros(4,1);
            A=rand(4,1);            % Loudness for each DutyCycle
            r=rand(4,1);            % Pulse emission rate for each DutyCycle
            v=zeros(4,1);           % Velocities
            F=zeros(4,1);           % Frequency
            P=zeros(4,1);           % Power
            Pfitness=zeros(4,1);
            DP=zeros(4,1);          % Dutycycles
               
            DP(1)=randi([5 250])/1000;
            DP(2)=randi([250 500])/1000;
            DP(3)=randi([500 750])/1000;
            DP(4)=randi([760 995])/1000;
                
            k=0;
    end
        
    Pprev = Vin*Iin;

    return;
else
    %update each particle's duty cycle, reset counter. 
    if(bool==0)
        D=DP(u);
    else
        D=DP(index); %mean          
    end
    DPcurrent=D;
    return; 
end    
end
