% based on the convergence of theta

handles.tol_rtheta=evalin('base','tol_rtheta');
n=handles.number_of_iteration;


rtheta_converge_flag=evalin('base','rtheta_converge_flag');
iter_rtheta_converged=evalin('base','iter_rtheta_converged');
iter_rtheta_conv=evalin('base','iter_rtheta_conv');
adjustment_of_stopping=evalin('base','adjustment_of_stopping');


handles.tol_rtheta
theta_est

if length(find(abs((theta_est(n,:))-(theta_est(n-1,:)))./abs((theta_est(n,:))) <handles.tol_rtheta)) == 4 && abs((theta_est(n,1))-(paramLB(1)))>0.1*sac && abs((theta_est(n,1))-(paramUB(1)))>0.1*sac && abs((theta_est(n,2))-(paramLB(2)))>0.1*sac && abs((theta_est(n,2))-(paramUB(2)))>0.1*sac && abs((theta_est(n,3))-(paramLB(3)))>0.1*sac && abs((theta_est(n,3))-(paramUB(3)))>.1*sac && abs((theta_est(n,4))-(paramLB(4)))>sac && abs((theta_est(n,4))-(paramUB(4)))>sac 
           
    rtheta_converge_flag(n) = 1
else
    rtheta_converge_flag(n) = 0
end

if n>4
    
 if adjustment_of_stopping==1 
   if rtheta_converge_flag(n)==1 %&& rtheta_converge_flag(n-1)==1 && rtheta_converge_flag(n-2)==1 && rtheta_converge_flag(n-3)==1 && rtheta_converge_flag(n-4)==1
    
   iter_rtheta_converged=[iter_rtheta_converged n];
   handles.iter_rtheta_converged=iter_rtheta_converged(end)-1;
   set(handles.edit12,'String',num2str(handles.iter_rtheta_converged));
   handles.Stopping_rule_satisfied='Yes';
   assignin('base','Stopping_rule_satisfied',handles.Stopping_rule_satisfied);
  
   answer=Modalbox1('Title','Termination');
   switch answer
       case 'Yes'
           
       case 'No' 
   end
   
   end

 elseif adjustment_of_stopping==2
   if rtheta_converge_flag(n)==1 && rtheta_converge_flag(n-1)==1 %&& rtheta_converge_flag(n-2)==1 && rtheta_converge_flag(n-3)==1 && rtheta_converge_flag(n-4)==1
   iter_rtheta_converged=[iter_rtheta_converged n];
   handles.iter_rtheta_converged=iter_rtheta_converged(end)-1;
   set(handles.edit12,'String',num2str(handles.iter_rtheta_converged));
    handles.Stopping_rule_satisfied='Yes';
   assignin('base','Stopping_rule_satisfied',handles.Stopping_rule_satisfied);
  
   answer=Modalbox1('Title','Termination');
   switch answer
       case 'Yes'
           
       case 'No' 
   end
   
   
   end
 
 elseif adjustment_of_stopping==3
   if rtheta_converge_flag(n)==1 && rtheta_converge_flag(n-1)==1 && rtheta_converge_flag(n-2)==1 %&& rtheta_converge_flag(n-3)==1 && rtheta_converge_flag(n-4)==1
   iter_rtheta_converged=[iter_rtheta_converged n];
   handles.iter_rtheta_converged=iter_rtheta_converged(end)-1;
   set(handles.edit12,'String',num2str(handles.iter_rtheta_converged));
    handles.Stopping_rule_satisfied='Yes';
   assignin('base','Stopping_rule_satisfied',handles.Stopping_rule_satisfied);
   
   answer=Modalbox1('Title','Termination');
   switch answer
       case 'Yes'
           
       case 'No' 
   end
   
   
   end
 
 elseif adjustment_of_stopping==4
   if rtheta_converge_flag(n)==1 && rtheta_converge_flag(n-1)==1 && rtheta_converge_flag(n-2)==1 && rtheta_converge_flag(n-3)==1 %&& rtheta_converge_flag(n-4)==1
   iter_rtheta_converged=[iter_rtheta_converged n];
   handles.iter_rtheta_converged=iter_rtheta_converged(end)-1;
   set(handles.edit12,'String',num2str(handles.iter_rtheta_converged));
    handles.Stopping_rule_satisfied='Yes';
   assignin('base','Stopping_rule_satisfied',handles.Stopping_rule_satisfied);
   
   answer=Modalbox1('Title','Termination');
   switch answer
       case 'Yes'
           
       case 'No' 
   end
   
   
   end
 
 elseif adjustment_of_stopping==5
   if rtheta_converge_flag(n)==1 && rtheta_converge_flag(n-1)==1 && rtheta_converge_flag(n-2)==1 && rtheta_converge_flag(n-3)==1 && rtheta_converge_flag(n-4)==1
   iter_rtheta_converged=[iter_rtheta_converged n];
   handles.iter_rtheta_converged=iter_rtheta_converged(end)-1;
   set(handles.edit12,'String',num2str(handles.iter_rtheta_converged));
    handles.Stopping_rule_satisfied='Yes';
   assignin('base','Stopping_rule_satisfied',handles.Stopping_rule_satisfied);
   
   answer=Modalbox1('Title','Termination');
   switch answer
       case 'Yes'
           
       case 'No' 
   end
   
   
   end
     
 end
end

handles.rtheta_converge_flag=rtheta_converge_flag;
assignin('base','rtheta_converge_flag',handles.rtheta_converge_flag);
handles.iter_rtheta_converged=iter_rtheta_converged;
assignin('base','iter_rtheta_converged',handles.iter_rtheta_converged);
handles.iter_rtheta_conv=iter_rtheta_conv;
assignin('base','iter_rtheta_conv',handles.iter_rtheta_conv);

guidata(hObject,handles);
