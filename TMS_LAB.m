function varargout = TMS_LAB(varargin)
% TMS_LAB MATLAB code for TMS_LAB.fig
%      TMS_LAB, by itself, creates a new TMS_LAB or raises the existing
%      singleton*.
%
%      H = TMS_LAB returns the handle to a new TMS_LAB or the handle to
%      the existing singleton*.
%
%      TMS_LAB('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TMS_LAB.M with the given input arguments.
%
%      TMS_LAB('Property','Value',...) creates a new TMS_LAB or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before TMS_LAB_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to TMS_LAB_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help TMS_LAB

% Last Modified by GUIDE v2.5 09-Oct-2018 11:40:39

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @TMS_LAB_OpeningFcn, ...
                   'gui_OutputFcn',  @TMS_LAB_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before TMS_LAB is made visible.
function TMS_LAB_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to TMS_LAB (see VARARGIN)

% Choose default command line output for TMS_LAB
handles.output = hObject;


%% prealication

handles.Total_runs=1;
assignin('base', 'Total_runs',handles.Total_runs)

handles.plot_animation = 1; %  To run saving animation, set this to 1
assignin('base', 'plot_animation',handles.plot_animation)

handles.save_er_plot_after_each_run=1; %save 
assignin('base', 'save_er_plot_after_each_run',handles.save_er_plot_after_each_run)

handles.total_theta_true=[];
assignin('base', 'total_theta_true',handles.total_theta_true)

handles.total_sigma_y=[];
assignin('base', 'total_sigma_y',handles.total_sigma_y)

handles.fmr=0.1;% threshold value for bad fit detection
assignin('base', 'fmr',handles.fmr)

handles.sac=0.1;% threshold value to detect whether estimation is saturated
assignin('base', 'sac',handles.sac)

%% Prealocation of the variables for evaluations

handles.theta1_F=[];
assignin('base', 'theta1_F',handles.theta1_F);

handles.theta2_F=[];
assignin('base', 'theta2_F',handles.theta2_F);

handles.theta3_F=[];
assignin('base', 'theta3_F',handles.theta3_F);

handles.theta4_F=[];
assignin('base', 'theta4_F',handles.theta4_F);

handles.er_theta1_F=[];
assignin('base', 'er_theta1_F',handles.er_theta1_F);

handles.er_theta1_F_conv=[];
assignin('base', 'er_theta1_F_conv',handles.er_theta1_F_conv);

handles.er_theta1_F_final_iter=[];
assignin('base', 'er_theta1_F_final_iter',handles.er_theta1_F_final_iter);

handles.er_theta2_F=[];
assignin('base', 'er_theta2_F',handles.er_theta2_F);

handles.er_theta2_F_conv=[];
assignin('base', 'er_theta2_F_conv',handles.er_theta2_F_conv);

handles.er_theta2_F_final_iter=[];
assignin('base', 'er_theta2_F_final_iter',handles.er_theta2_F_final_iter);

handles.er_theta3_F=[];
assignin('base', 'er_theta3_F',handles.er_theta3_F);

handles.er_theta3_F_conv=[];
assignin('base', 'er_theta3_F_conv',handles.er_theta3_F_conv);

handles.er_theta3_F_final_iter=[];
assignin('base', 'er_theta3_F_final_iter',handles.er_theta3_F_final_iter);


handles.er_theta4_F=[];
assignin('base', 'er_theta4_F',handles.er_theta4_F);

handles.er_theta4_F_conv=[];
assignin('base', 'er_theta4_F_conv',handles.er_theta4_F_conv);

handles.er_theta4_F_final_iter=[];
assignin('base', 'er_theta4_F_final_iter',handles.er_theta4_F_final_iter);

handles.no_of_runs_F_conv=0;
assignin('base', 'no_of_runs_F_conv',handles.no_of_runs_F_conv);

handles.theta_F_conv=[];
assignin('base', 'theta_F_conv',handles.theta_F_conv);

handles.iter_F_conv=[];
assignin('base', 'iter_F_conv',handles.iter_F_conv);

handles.outSim=0; %record saturated runs; if it is empty --> runs are OK
assignin('base', 'outSim',handles.outSim);

handles.xid_new=[];
assignin('base', 'xid_new',handles.xid_new);

handles.xid_byFIM=[];% this variable my be usless and removable!
assignin('base', 'xid_byFIM',handles.xid_byFIM);

handles.xid_ga=[];% input computed by genetic algorithm 
assignin('base', 'xid_ga',handles.xid_ga);

handles.xid_ms=[];% input computed by multiple start
assignin('base', 'xid_ms',handles.xid_ms);

handles.xid_gs=[];% input computed by global search
assignin('base', 'xid_gs',handles.xid_gs);

handles.fval_gs=[];
assignin('base', 'fval_gs',handles.fval_gs);

handles.fval_ga=[];% cost function genetic algorithm ; cost_func=det(FIM)
assignin('base', 'fval_ga',handles.fval_ga);

handles.fval_ms=[];
assignin('base', 'fval_ms',handles.fval_ms);

handles.yid_new=[]; % output vector asscoiated xid_new,
assignin('base', 'yid_new',handles.yid_new);

handles.yid_paper=[];% used for plot
assignin('base', 'yid_paper',handles.yid_paper);

handles.theta_est=[0 0 0 0;0 0 0 0];
assignin('base', 'theta_est',handles.theta_est);

handles.var_theta_est=[]; % variance of estimations;for the computation of Cramer-Rao bound
assignin('base', 'var_theta_est',handles.var_theta_est);

handles.yest_xid=[]; % output of model by using FIM-SPE estimation results and xid 
assignin('base', 'yest_xid',handles.yest_xid);

handles.CRB_true=[]; % Cramer-Rao bounds computed using theta_est
assignin('base', 'CRB_true',handles.CRB_true);

handles.var_CRB=[]; 
assignin('base', 'var_CRB',handles.var_CRB);

handles.rtheta_converge_flag=[]; % Memory of how many consequative convergence happend
assignin('base', 'rtheta_converge_flag',handles.rtheta_converge_flag);   % It is used in SS_stopping_rtheta.m

handles.er_theta=[];% Absolute Relative Error (ARE) using FIM-SPE
assignin('base', 'er_theta',handles.er_theta);
%% Prealocation added 16Aug-2018

handles.nxeq0=0;
assignin('base','nxeq0',handles.nxeq0);

handles.xeq0=zeros(1,handles.nxeq0);
assignin('base','xeq0',handles.xeq0);

handles.sigma_y=0.0001;
assignin('base', 'sigma_y',handles.sigma_y);

handles.y_var0=0;
assignin('base','y_var0',handles.y_var0);

%% Prealocation from getparameter

handles.xid_min=0;
assignin('base', 'xid_min',handles.xid_min);
%handles.xid_min=evalin('base', 'xid_min');

handles.xid_max=1;
assignin('base', 'xid_max',handles.xid_max);
%handles.xid_max=evalin('base', 'xid_max');

handles.yl_true=1e-6 + (1e-5 - 1e-6).*rand;
assignin('base', 'yl_true',handles.yl_true);
%handles.yl_true=evalin('base', 'yl_true');

handles.yh_true=1e-4 + (1e-2 - 1e-4).*rand;
assignin('base', 'yh_true',handles.yh_true);
%handles.yh_true=evalin('base', 'yh_true');


handles.m_true=.1 + (0.9-.1)*handles.xid_max*rand;
assignin('base', 'm_true',handles.m_true);
%handles.m_true=evalin('base', 'm_true');

  if handles.m_true <.2 || handles.m_true >= 0.8
            handles.s_true=30 + (50-30)*rand;
        elseif handles.m_true >= .2 && handles.m_true < 0.3
            handles.s_true=20 + (40-20)*rand;
        elseif handles.m_true >= .7 && handles.m_true < 0.8
            handles.s_true=20 + (40-20)*rand;
        elseif handles.m_true >= .3 && handles.m_true < 0.45
            handles.s_true=10 + (30-10)*rand;
        elseif handles.m_true >= .55 && handles.m_true < 0.7
            handles.s_true=10 + (30-10)*rand;
        else
            handles.s_true=5 + (20-5)*rand;
  end
             assignin('base', 's_true',handles.s_true);


handles.theta_true=[handles.yl_true handles.yh_true handles.m_true handles.s_true];
assignin('base', 'theta_true',handles.theta_true);


%% plot true model 2
%  Plot Data that is geenrated using Update GUI
        rng('shuffle');%seeds the random number generator based on the current time.
                       %Thus, rand, randi, and randn produce a different sequence of

         handles.x_dg=linspace(handles.xid_min,handles.xid_max,10000); 
         assignin('base', 'x_dg',handles.x_dg);

         handles.theta_true=[handles.yl_true;handles.yh_true;handles.m_true;handles.s_true];
         assignin('base', 'theta_true',handles.theta_true);
         
         handles.y_var=handles.sigma_y*randn(1,length(handles.x_dg));
         assignin('base', 'y_var',handles.y_var);
         
         handles.yt_true=SSsigmoidFunc(handles.x_dg,handles.theta_true,0);
         assignin('base', 'yt_true',handles.yt_true);
         
         handles.yt_noise=SSsigmoidFunc(handles.x_dg,handles.theta_true,0);
         assignin('base', 'yt_noise',handles.yt_noise);
         
         handles.logyt_m=log(handles.yt_noise)+handles.y_var;
         assignin('base', 'logyt_m',handles.logyt_m);
         
%% basic curve fitting 3

 handles.eyL_th=100;% for fit modification
 assignin('base', 'eyL_th',handles.eyL_th);
 
 handles.em_th=150;
 assignin('base', 'eyL_th',handles.eyL_th);
 
 handles.Weight_gain1=1;%1e2;
 assignin('base', 'Weight_gain1',handles.Weight_gain1);      
  
 handles.Weight_gain2=1;
 assignin('base', 'Weight_gain2',handles.Weight_gain2);
        
 handles.WG_th=.5;
 assignin('base', 'WG_th',handles.WG_th);
 
 handles.bias=0;%1e-3;
 assignin('base', 'bias',handles.bias);
        
 handles.no_ini_samples_fim=4;
 assignin('base', 'no_ini_samples_fim',handles.no_ini_samples_fim);
        
 handles.no_next_iter=150;
 assignin('base', 'no_next_iter',handles.no_next_iter);
        
 handles.total_no_iterations=handles.no_ini_samples_fim+handles.no_next_iter;
 assignin('base', 'total_no_iterations',handles.total_no_iterations);
 
 handles.Nmax=handles.total_no_iterations;
 assignin('base', 'Nmax',handles.Nmax);
 
 handles.paramLB=[-7 -4 .01 .01];% lower limit of estimation, applies when trust-region method is used
 assignin('base', 'paramLB',handles.paramLB);
   
 handles.paramUB=[-3 -1 1 100];% upper limit of estimation
 assignin('base', 'paramUB',handles.paramUB);
   
 handles.xval=linspace(0,handles.xid_max,100);% for validation of estimated models
 assignin('base', 'xval',handles.xval); 
 
 handles.size_val=length(handles.xval);
 assignin('base', 'size_val',handles.size_val);
 
        handles.success_stop=0;
         assignin('base', 'success_stop',handles.success_stop); 
         
        handles.sucess_flag=0;
         assignin('base', 'sucess_flag',handles.sucess_flag); 
         
        handles.xid=[];
         assignin('base', 'xid',handles.xid); 
         
        handles.yid=[];
         assignin('base', 'yid',handles.yid); 
         
        handles.yest_val=[]; % output of model by using FIM-SPE estimation results and xid_val
         assignin('base', 'yest_val',handles.yest_val); 
        
        handles.fit_modification_flag=[];
         assignin('base', 'fit_modification_flag',handles.fit_modification_flag); 
         
        handles.WG=[];% Weightings for curve fittinG
         assignin('base', 'WG',handles.WG); 
         
        handles.WG_gain=[];
         assignin('base', 'WG_gain',handles.WG_gain); 
         
        handles.CRB=[];
         assignin('base', 'CRB',handles.CRB); 
         
         handles.FIM=[];
         assignin('base', 'FIM',handles.FIM); 
         
        handles.var_theta_est(handles.no_ini_samples_fim,:)=[0 0 0 0];
         assignin('base', 'var_theta_est',handles.var_theta_est); 
         
         handles.func_type=1;      % choose 0 to estimate the actual function, and 1 to estimate the logarithm of the function  
         assignin('base', 'func_type',handles.func_type); 
         
         handles.initial_setting=0;% curve fitting initilaization method: choose 0 for random initialization and 1 for using the most recent estimate  
         assignin('base', 'initial_setting',handles.initial_setting); 
  
         handles.opt_method=1;     % Choose the optimization algorithm of FIM (0 Genetic Algorithm | 1 GlobalSearch-default | 2 MultiStart)
          assignin('base', 'opt_method',handles.opt_method);   
         
         handles.same_xid_ini=1;
          assignin('base', 'same_xid_ini',handles.same_xid_ini); 
 
         handles.rtheta_converge_flag(1:4)=[0 0 0 0];
          assignin('base', 'rtheta_converge_flag',handles.rtheta_converge_flag); 
        
         handles.iter_rtheta_converged=[];
          assignin('base', 'iter_rtheta_converged',handles.iter_rtheta_converged); 
          
         handles.iter_rtheta_conv=[];
          assignin('base', 'iter_rtheta_conv',handles.iter_rtheta_conv); 
          
         handles.xid_ini=[];
           assignin('base', 'xid_ini',handles.xid_ini);   
           
         handles.n=[];
            %assignin('base', 'n',handles.n);   
            
         handles.Session_Data=[];
         assignin('base','Session_Data',handles.Session_Data);
         
         handles.Stopping_rule_satisfied='No';
         assignin('base','Stopping_rule_satisfied', handles.Stopping_rule_satisfied);
         
         handles.Subject_Information=[];
         assignin('base','Subject_Information',handles.Subject_Information);
         
         handles.Session_Number=[];
         assignin('base','Session_Number',handles.Session_Number);
         
         handles.Session_Date=[];
         assignin('base','Session_Date',handles.Session_Date);

%% FIM_estimation4
handles.number_of_iteration=5;

%% Update_estimation5
handles.yid_newGUI=[];
handles.i_xidnew=[];
handles.number_of_push=0;
assignin('base','number_of_push',handles.number_of_push);
handles.xid_paper=[];
handles.logyid_new=[];
handles.xData=[];
handles.pdpnts=[];
handles.yData=[];
handles.yidtemp=[];
handles.pdpnts=[];
handles.pnewfit=[];
handles.xidnm1=[];
handles.yidnm1=[];
handles.yest_xid=[];
handles.yest_val=[];

handles.pdpnts=[];
assignin('base','pdpnts',handles.pdpnts);
handles.poldfit=[];
assignin('base','poldfit',handles.poldfit);
handles.pnewpoint=[];
assignin('base','pnewpoint',handles.pnewpoint);
handles.pnewfit=[];
assignin('base','pnewfit',handles.pnewfit);
handles.thereshold_value=[];
assignin('base','thereshold_value',handles.thereshold_value);
%%
handles.table_var=[0 0 0 0;0 0 0 0];
assignin('base','table_var',handles.table_var);
%% Preallocation of Recording Data from Baseline 17Aug-2018

handles.initial=zeros(2,handles.nxeq0);
assignin('base','initial',handles.initial);

handles.BaselineDate=[];
assignin('base','BaselineDate',handles.BaselineDate);

handles.xfim_gs=[];
assignin('base','xfim_gs',handles.xfim_gs);

handles.xid_fim_baseline=[];
assignin('base','xid_fim_baseline',handles.xid_fim_baseline);

handles.yid_fim_baseline=[];
assignin('base','yid_fim_baseline',handles.yid_fim_baseline);

handles.initialization=zeros(2,3);
assignin('base','initialization',handles.initialization);

handles.iter_ation=0;
assignin('base','iter_ation',handles.iter_ation);

handles.adjustment_of_stopping(1)=5;
assignin('base','adjustment_of_stopping',handles.adjustment_of_stopping);

% slider (number of stopping rule) 
set(handles.slider1,'Min',1);
set(handles.slider1,'Max',5);
set(handles.slider1,'Value',5);
set(handles.slider1,'SliderStep',[0.25,0.25]);
set(handles.slider1,'Position',[3.5 0.5 25 1.9]);
handles.output=hObject;

% tollerance of parameters
handles.tol_rtheta=[0.01 0.01 0.01 0.01];
assignin('base','tol_rtheta',handles.tol_rtheta);

%Preallocation of Saving

handles.Name1=[];
assignin('base','Name1',handles.Name1)

handles.Name2=[];
assignin('base','Name2',handles.Name2)

handles.Name3=[];
assignin('base','Name3',handles.Name3)

handles.Name4=[];
assignin('base','Name4',handles.Name4)

handles.Name5=[];
assignin('base','Name5',handles.Name5)

handles.Name6=[];
assignin('base','Name6',handles.Name6)

handles.Name7=[];
assignin('base','Name7',handles.Name7)

handles.Name8=[];
assignin('base','Name8',handles.Name8)

handles.Name9=[];
assignin('base','Name9',handles.Name9)

handles.Name10=[];
assignin('base','Name10',handles.Name10)

handles.Name11=[];
assignin('base','Name11',handles.Name11)

handles.Name12=[];
assignin('base','Name12',handles.Name12)

handles.Name13=[];
assignin('base','Name13',handles.Name13)

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes TMS_LAB wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = TMS_LAB_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double
str = get(hObject,'String');
handles.yid_newGUI=str2double(str);
handles.yid_newGUI=handles.yid_newGUI*0.000001;
yid_fim_baseline=evalin('base','yid_fim_baseline');
handles.yid_fim_baseline=[yid_fim_baseline handles.yid_newGUI];
assignin('base','yid_fim_baseline',handles.yid_fim_baseline);
assignin('base', 'yid_newGUI',handles.yid_newGUI)


% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double
guidata(hObject,handles);



% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% leila
% handles.number_of_iteration=handles.number_of_iteration+1;
% assignin('base', 'number_of_iteration',handles.number_of_iteration);

if handles.number_of_push==0
   
    % exporting value from start bottom to workspace- Recording Baseline
handles.iter_ation=3;
set(handles.edit31,'String',num2str(handles.iter_ation));
assignin('base','iter_ation',handles.iter_ation);
        
handles.nxeq0=evalin('base','nxeq0');

handles.xeq0=zeros(1,handles.nxeq0);
assignin('base','xeq0',handles.xeq0);

handles.sigma_y=0.1 + (1-.1).*rand;
assignin('base', 'sigma_y',handles.sigma_y);

handles.y_var0=evalin('base','y_var0');


handles.initial(1,1:(handles.nxeq0+3))=[handles.xeq0(1,1:handles.nxeq0) handles.initialization(1,:)];
handles.initial(2,1:(handles.nxeq0+3))=[handles.y_var0(1,1:handles.nxeq0) handles.initialization(2,:)];
assignin('base', 'initial',handles.initial);
initial=evalin('base','initial');
handles.yid_fim_baseline=initial(2,:);
assignin('base','yid_fim_baseline',handles.yid_fim_baseline);

handles.xid_fim_baseline=initial(1,:);
assignin('base','xid_fim_baseline',handles.xid_fim_baseline)

initial=handles.initial;

%% plot true model
%  Leila: Delete ploting true model  17Aug-2018
         axes(handles.axes1);   
      if handles.plot_animation == 1 
          hold on
          xlabel('Pulse Amplitude (%MO)')
          ylabel('MEP (µV)')
          %title('FIM SPE')
          ax1=gca;
          ax1.FontName = 'Times New Roman';
          ax1.FontSize = 10;
          ax1.XLim=[0 1];
          ax1.YScale='log';
          %AX=legend('previous samples','new sample','previous fit','new fit');
          handles.test_d=[handles.theta_true';zeros(1,4);
          zeros(1,4)];
       end

%% plot curve fitting
  handles.n=handles.nxeq0;
  
  SS_ini_FIM_GUI; 
            %%
      if plot_animation == 1 
       
        hold on
        % pdpnt plot data points
        yest_val=evalin('base', 'yest_val');
        xid_fim_baseline=evalin('base','xid_fim_baseline');
        yid_fim_baseline=evalin('base','yid_fim_baseline');
        yid_fim_baseline=1000000*(yid_fim_baseline);
        
        pdpnts=plot(xid_fim_baseline(1:end-3),yid_fim_baseline(1:end-3),...
            'LineStyle','none',...
            'LineWidth',1,...
            'Marker','d',...
            'MarkerSize',8,...
            'MarkerEdgeColor','m',...
            'MarkerFaceColor','none');
          
         pdpnts=plot(xid_fim_baseline(end-2:end),yid_fim_baseline(end-2:end),...
            'LineStyle','none',...
            'LineWidth',1,...
            'Marker','o',...
            'MarkerSize',8,...
            'MarkerEdgeColor','r',...
            'MarkerFaceColor','none');
        
           pnewfit=plot(handles.xval,1e6*10.^(yest_val(m,1:end)),...
            'LineStyle','-',...
            'LineWidth',1,...
            'Color','r');
        AX=legend('Baseline sample','IO Curve Estimation','location','northwest');
        handles.yest_val=yest_val;
        assignin('base','yest_val',handles.yest_val);
        handles.pdpnts=pdpnts;
        assignin('base','pdpnts',handles.pdpnts);
        handles.pnewfit=pnewfit;
        assignin('base','pnewfit',handles.pnewfit);
          
        %% parameters
        
            evalin('base','theta_est(4,1)');
            handles.theta1_F(n)=theta_est(4,1);
            assignin('base','theta1_F',handles.theta1_F);
            
            evalin('base','theta_est(4,2)');
            handles.theta2_F(n)=theta_est(4,2);
            assignin('base','theta2_F',handles.theta2_F);
            
            evalin('base','theta_est(4,3)');
            handles.theta3_F(n)=theta_est(4,3);
            assignin('base','theta3_F',handles.theta3_F);
            
            evalin('base','theta_est(4,4)');
            handles.theta4_F(n)=theta_est(4,4);
            assignin('base','theta4_F',handles.theta4_F);
          

      end
      

%% FIM_estimation4
handles.handles.number_of_iteration=5;

xid= evalin('base','xid');
yid= evalin('base','y_var0');
handles.yid=yid;
                handles.xidnm1=xid;% to use for plot
                handles.yidnm1=yid;% to use for plot
                FIM=evalin('base','FIM');
                handles.FIM=FIM;
                assignin('base','FIM',handles.FIM);
                
                assignin('base','xidnm1',handles.xidnm1);
                assignin('base','yidnm1',handles.yidnm1);
    
    
                nxeq0=evalin('base','nxeq0');
                
               theta_est(end,:)
                ObjFunc_FIM = @(x) SSfim_cost_modified(0,m-n,x,initial(1:3),theta_est(end,:),[],1);% 
                optsFIM = optimoptions(@fmincon,'Algorithm','interior-point');
                problem = createOptimProblem('fmincon','x0',handles.xid_max*rand,...
                    'objective',ObjFunc_FIM,'lb',0.1,'ub',handles.xid_max,'options',optsFIM);
                
                         
                if handles.opt_method == 0 % ga was chosen
                    [xid_ga(handles.number_of_iteration),fval_ga(handles.number_of_iteration)] = ga(ObjFunc_FIM,1,[],[],[],[],xid_min,xid_max);
                    xid_new(handles.number_of_iteration)  = xid_ga(handles.number_of_iteration);
                    fval_x(handles.number_of_iteration)   =fval_ga(handles.number_of_iteration);
                elseif handles.opt_method == 1 % Global search was chosen
                    [xid_gs(handles.number_of_iteration),fval_gs(handles.number_of_iteration),flagm_gs,outptm_gs,manyminsm_gs] = run(GlobalSearch,problem);
                    xid_new(handles.number_of_iteration)  = xid_gs(handles.number_of_iteration);
                    handles.xfim_gs=xid_new(handles.number_of_iteration);
                    assignin('base','xfim_gs',handles.xfim_gs);
                   
                    fval_x(handles.number_of_iteration)   =fval_gs(handles.number_of_iteration);
                else   % MultiStartPoint was chosen
                    pts = xid_min + (xid_max-xid_min).*rand(200,1);
                    tpoints = CustomStartPointSet(pts);
                    allpts = {tpoints};
                
                    [xid_ms(handles.number_of_iteration),fval_ms(handles.number_of_iteration),flagm_ms,outptm_ms,manyminsm_ms] = run(MultiStart,problem,allpts);
                    xid_new(handles.number_of_iteration)  = xid_ms(handles.number_of_iteration);
                    fval_x(handles.number_of_iteration)   =fval_ms(handles.number_of_iteration);
                end
                
               
                handles.xid_fim_baseline=[initial(1,:) handles.xfim_gs];
                assignin('base','xid_fim_baseline',handles.xid_fim_baseline);
              
                handles.xid_new=round(xid_new,2);
                
                handles.xid_byFIM(handles.number_of_iteration)=handles.xid_new(handles.number_of_iteration);
                assignin('base', 'xid_new',handles.xid_new);
                handles.xid_new(handles.number_of_iteration)=100*(handles.xid_new(handles.number_of_iteration));
                set(handles.edit9,'String',num2str(handles.xid_new(handles.number_of_iteration)));
                assignin('base', 'number_of_iteration',handles.number_of_iteration);
                %assignin('base','xid',handles.xid);
               handles.number_of_push=handles.number_of_push+1;
     
elseif handles.number_of_push~=0

 handles.number_of_push=handles.number_of_push+1;
 assignin('base','number_of_push',handles.number_of_push);
if  (handles.number_of_push)/2+2 < handles.no_next_iter
 
 if mod(handles.number_of_push,2)==0
     bgClr = get(handles.pushbutton6,'BackgroundColor');
     set(handles.pushbutton6,'BackgroundColor',[1 0 0]);
    
 push6;
 %set(handles.uitable1,'Data',table_var);
 set(handles.pushbutton6,'BackgroundColor',bgClr);


 else
    
     bgClr = get(handles.pushbutton6,'BackgroundColor');
     set(handles.pushbutton6,'BackgroundColor',[0.16 0.27 0.16]);
    
     handles.number_of_iteration=5+(handles.number_of_push-1)/2;
     assignin('base','number_of_iteration',handles.number_of_iteration);
      n=handles.number_of_iteration;

            theta_est=evalin('base','theta_est');
            
          
            evalin('base','theta_est(4,1)');
            handles.theta1_F(n)=theta_est(4,1);
            assignin('base','theta1_F',handles.theta1_F);
            
            evalin('base','theta_est(4,2)');
            handles.theta2_F(n)=theta_est(4,2);
            assignin('base','theta2_F',handles.theta2_F);
            
            evalin('base','theta_est(4,3)');
            handles.theta3_F(n)=theta_est(4,3);
            assignin('base','theta3_F',handles.theta3_F);
            
            evalin('base','theta_est(4,4)');
            handles.theta4_F(n)=theta_est(4,4);
            assignin('base','theta4_F',handles.theta4_F);
              %%%% FIM_estimation4

              xid_fim_baseline=evalin('base','xid_fim_baseline');
              yid_fim_baseline=evalin('base','yid_fim_baseline');
  
                handles.xidnm1=xid_fim_baseline;% to use for plot
                handles.yidnm1=yid_fim_baseline;% to use for plot

                
                assignin('base','xidnm1',handles.xidnm1);
                assignin('base','yidnm1',handles.yidnm1);
    
                var_theta_est=evalin('base','var_theta_est');
                FIM=evalin('base','FIM');
                no_ini_samples_fim=evalin('base','no_ini_samples_fim');  
                sigma_y=evalin('base','sigma_y');
                
               
                theta_est(end,:);
                ObjFunc_FIM = @(x) SSfim_cost_modified(0,n-handles.nxeq0,x,xid_fim_baseline(handles.nxeq0+1:end),theta_est(end,:),[],1);
                optsFIM = optimoptions(@fmincon,'Algorithm','interior-point');
                problem = createOptimProblem('fmincon','x0',handles.xid_max*rand,...
                    'objective',ObjFunc_FIM,'lb',0.1,'ub',handles.xid_max,'options',optsFIM);
                handles.FIM=FIM;
                assignin('base','FIM',handles.FIM);
                
                      
                if handles.opt_method == 0 % ga was chosen
                    [xid_ga(handles.number_of_iteration),fval_ga(handles.number_of_iteration)] = ga(ObjFunc_FIM,1,[],[],[],[],xid_min,xid_max);
                    xid_new(handles.number_of_iteration)  = xid_ga(handles.number_of_iteration);
                    fval_x(handles.number_of_iteration)   =fval_ga(handles.number_of_iteration);
                elseif handles.opt_method == 1 % Global search was chosen
                    [xid_gs(handles.number_of_iteration),fval_gs(handles.number_of_iteration),flagm_gs,outptm_gs,manyminsm_gs] = run(GlobalSearch,problem);
                    xid_new(handles.number_of_iteration)  = xid_gs(handles.number_of_iteration);
                    handles.xfim_gs=xid_new(handles.number_of_iteration);
                    assignin('base','xfim_gs',handles.xfim_gs);
                    fval_x(handles.number_of_iteration)   =fval_gs(handles.number_of_iteration);
                else   % MultiStartPoint was chosen
                    pts = xid_min + (xid_max-xid_min).*rand(200,1);
                    tpoints = CustomStartPointSet(pts);
                    allpts = {tpoints};
                
                    [xid_ms(handles.number_of_iteration),fval_ms(handles.number_of_iteration),flagm_ms,outptm_ms,manyminsm_ms] = run(MultiStart,problem,allpts);
                    xid_new(handles.number_of_iteration)  = xid_ms(handles.number_of_iteration);
                    fval_x(handles.number_of_iteration)   =fval_ms(handles.number_of_iteration);
                end
                handles.xid_new(handles.number_of_iteration)=round(xid_new(handles.number_of_iteration),2);
                handles.xid_byFIM(handles.number_of_iteration)=handles.xid_new(handles.number_of_iteration);
                assignin('base', 'xid_new',handles.xid_new);
                handles.xid_new(handles.number_of_iteration)=(100).*(handles.xid_new(handles.number_of_iteration));
                set(handles.edit9,'String',num2str(handles.xid_new(handles.number_of_iteration)));
                xid_fim_baseline=evalin('base','xid_fim_baseline');
                handles.xid_fim_baseline=[xid_fim_baseline handles.xfim_gs];
                assignin('base','xid_fim_baseline',handles.xid_fim_baseline);
                
                set(handles.edit10,'String',num2str(0));
                assignin('base', 'number_of_iteration',handles.number_of_iteration);
                set(handles.pushbutton6,'BackgroundColor',bgClr);


 end

else  bgClr = get(handles.pushbutton6,'BackgroundColor');
     %set(handles.pushbutton6,'BackgroundColor',[0 1 0]);
     no_next_iter=evalin('base','no_next_iter');
     warndlg('Maximum number of pulses was taken, and the session is terminated!','Maximum Number Of Pulses');
end
end
%GUI_Copy_of_SS_Main_MahdiV4T5
guidata(hObject,handles);





function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
str = get(hObject,'String');
handles.Subject_Information=(str);
assignin('base','Subject_Information',handles.Subject_Information);
guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
% str = get(hObject,'String');
% handles.nxeq1=str2double(str);
% assignin('base', 'nxeq0',handles.nxeq0);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

 

% --- Executes when entered data in editable cell(s) in uitable1.
function uitable1_CellEditCallback(hObject, eventdata, handles)
% hObjec t    handle to uitable1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
set(hObject,'Data',handles.theta_est(end-1:end,:));
disp(tables);
guidata(hObject,handles);



function edit20_Callback(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit20 as text
%        str2double(get(hObject,'String')) returns contents of edit20 as a double
str = get(hObject,'String');
handles.Session_Number=(str);
assignin('base','Session_Number',handles.Session_Number);
guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function edit20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit21_Callback(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit21 as text
%        str2double(get(hObject,'String')) returns contents of edit21 as a double
str = get(hObject,'String');
handles.Session_Date=(str);
assignin('base','Session_Date',handles.Session_Date);
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit22_Callback(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit22 as text
%        str2double(get(hObject,'String')) returns contents of edit22 as a double
str = get(hObject,'String');
handles.y_var0=str2num(str);
handles.y_var0=handles.y_var0*0.000001;
handles.y_var0
assignin('base', 'y_var0',handles.y_var0);
handles.nxeq0=length(handles.y_var0);
assignin('base','nxeq0',handles.nxeq0);
set(handles.edit2,'String',num2str(handles.nxeq0));
guidata(hObject,handles);




% --- Executes during object creation, after setting all properties.
function edit22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit25_Callback(hObject, eventdata, handles)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit25 as text
%        str2double(get(hObject,'String')) returns contents of edit25 as a double
str = get(hObject,'String');
handles.initialization(1,1)=str2double(str);
handles.initialization(1,1)=handles.initialization(1,1)*(0.01);
assignin('base','initialization',handles.initialization);
guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function edit25_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit26_Callback(hObject, eventdata, handles)
% hObject    handle to edit26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit26 as text
%        str2double(get(hObject,'String')) returns contents of edit26 as a double
str = get(hObject,'String');
handles.initialization(2,1)=str2double(str);
handles.initialization(2,1)=handles.initialization(2,1)*0.000001;
assignin('base','initialization',handles.initialization);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit26_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit27_Callback(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit27 as text
%        str2double(get(hObject,'String')) returns contents of edit27 as a double
str = get(hObject,'String');
handles.initialization(1,2)=str2double(str);
handles.initialization(1,2)=handles.initialization(1,2)*(0.01);
assignin('base','initialization',handles.initialization);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit27_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit28_Callback(hObject, eventdata, handles)
% hObject    handle to edit28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit28 as text
%        str2double(get(hObject,'String')) returns contents of edit28 as a double
str = get(hObject,'String');
handles.initialization(2,2)=str2double(str);
handles.initialization(2,2)=handles.initialization(2,2)*0.000001;
assignin('base','initialization',handles.initialization);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit28_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit29_Callback(hObject, eventdata, handles)
% hObject    handle to edit29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit29 as text
%        str2double(get(hObject,'String')) returns contents of edit29 as a double
str = get(hObject,'String');
handles.initialization(1,3)=str2double(str);
handles.initialization(1,3)=handles.initialization(1,3)*(0.01);
assignin('base','initialization',handles.initialization);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit29_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit30_Callback(hObject, eventdata, handles)
% hObject    handle to edit30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit30 as text
%        str2double(get(hObject,'String')) returns contents of edit30 as a double
str = get(hObject,'String');
handles.initialization(2,3)=str2double(str);
handles.initialization(2,3)=handles.initialization(2,3)*0.000001;
assignin('base','initialization',handles.initialization);
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit30_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit30 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on uitable1 and none of its controls.
function uitable1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to uitable1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on mouse press over figure background.
function figure1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
delete(hObject);



function edit31_Callback(hObject, eventdata, handles)
% hObject    handle to edit31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit31 as text
%        str2double(get(hObject,'String')) returns contents of edit31 as a double


% --- Executes during object creation, after setting all properties.
function edit31_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%set(handles.slider1,'Min',1);
%set(handles.slider1,'Max',5);
%set(handles.slider1,'value',1);
%set(handles.slider1,'SliderStep',[0.25,0.25]);
%handles.output=h0bject;

handles=guidata(hObject);
newVal=get(handles.slider1,'Value');
newVal=round(newVal(1));
h0bject.Value=newVal;
set(handles.slider1,'value',newVal);
handles.adjustment_of_stopping=get(handles.slider1,'value');
assignin('base','adjustment_of_stopping',handles.adjustment_of_stopping);
set(handles.edit43,'String',num2str(newVal));
guidata(hObject, handles);



% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit32_Callback(hObject, eventdata, handles)
% hObject    handle to edit32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit32 as text
%        str2double(get(hObject,'String')) returns contents of edit32 as a double

handles.xid_max=str2double(get(hObject,'String'));
handles.xid_max=(0.01)*(handles.xid_max);
assignin('base','xid_max',handles.xid_max);
handles.paramUB(3)=handles.xid_max;
assignin('base','paramUB',handles.paramUB);
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit32_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit33_Callback(hObject, eventdata, handles)
% hObject    handle to edit33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit33 as text
%        str2double(get(hObject,'String')) returns contents of edit33 as a double

handles.no_next_iter= str2double(get(hObject,'String'));
 assignin('base', 'no_next_iter',handles.no_next_iter);
 guidata(hObject, handles);
 


% --- Executes during object creation, after setting all properties.
function edit33_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.xid_max=evalin('base','xid_max');
set(handles.edit25,'String',num2str(100*handles.xid_max));
handles.initialization(1,1)=handles.xid_max;
assignin('base','initialization',handles.initialization);
guidata(hObject, handles);


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.xid_max=evalin('base','xid_max');
set(handles.edit27,'String',num2str(round(100*(0.75*handles.xid_max))));
handles.initialization(1,2)=0.01*round(100*(0.75*handles.xid_max));
assignin('base','initialization',handles.initialization);
guidata(hObject, handles);



% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.xid_max=evalin('base','xid_max');
set(handles.edit29,'String',num2str(round(100*(0.5*handles.xid_max))));
handles.initialization(1,3)=0.01*round(100*(0.5*handles.xid_max));
assignin('base','initialization',handles.initialization);
guidata(hObject, handles);


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

%handles=guidata(hObject);
newVal_s=get(handles.slider3,'Value');
newVal_s=round((newVal_s*1000))*0.001;
%h0bject.Value=newVal;
set(handles.slider3,'Value',newVal_s);
%handles.tol_rtheta(1,4)=get(handles.slider3,'Value');
handles.tol_rtheta(1,4)=newVal_s;
assignin('base','tol_rtheta',handles.tol_rtheta);
set(handles.text37,'String',num2str(newVal_s));
 
guidata(hObject, handles);





% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
newVal_m=get(handles.slider4,'Value');
newVal_m=round((newVal_m*1000))*0.001;
set(handles.slider4,'Value',newVal_m);
handles.tol_rtheta(1,3)=newVal_m;
assignin('base','tol_rtheta',handles.tol_rtheta);
set(handles.text36,'String',num2str(newVal_m));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
newVal_yh=get(handles.slider5,'Value');
newVal_yh=round((newVal_yh*1000))*0.001;
%h0bject.Value=newVal;
set(handles.slider5,'Value',newVal_yh);
%handles.tol_rtheta(1,4)=get(handles.slider3,'Value');
handles.tol_rtheta(1,2)=newVal_yh;
assignin('base','tol_rtheta',handles.tol_rtheta);
set(handles.text35,'String',num2str(newVal_yh));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
newVal_yl=get(handles.slider6,'Value');
newVal_yl=round((newVal_yl*1000))*0.001;
%h0bject.Value=newVal;
set(handles.slider6,'Value',newVal_yl);
%handles.tol_rtheta(1,4)=get(handles.slider3,'Value');
handles.tol_rtheta(1,1)=newVal_yl;
assignin('base','tol_rtheta',handles.tol_rtheta);
set(handles.text34,'String',num2str(newVal_yl));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit39_Callback(hObject, eventdata, handles)
% hObject    handle to edit39 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit39 as text
%        str2double(get(hObject,'String')) returns contents of edit39 as a double


tolVal_yl=str2double(get(hObject,'String'));
handles.tol_rtheta(1,1)=tolVal_yl;
assignin('base','tol_rtheta',handles.tol_rtheta);

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit39_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit39 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit40_Callback(hObject, eventdata, handles)
% hObject    handle to edit40 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit40 as text
%        str2double(get(hObject,'String')) returns contents of edit40 as a double
tolVal_yh=str2double(get(hObject,'String'));
handles.tol_rtheta(1,2)=tolVal_yh;
assignin('base','tol_rtheta',handles.tol_rtheta);

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit40_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit40 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit41_Callback(hObject, eventdata, handles)
% hObject    handle to edit41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit41 as text
%        str2double(get(hObject,'String')) returns contents of edit41 as a double
tolVal_m=str2double(get(hObject,'String'));
handles.tol_rtheta(1,3)=tolVal_m;
assignin('base','tol_rtheta',handles.tol_rtheta);

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit41_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit42_Callback(hObject, eventdata, handles)
% hObject    handle to edit42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit42 as text
%        str2double(get(hObject,'String')) returns contents of edit42 as a double
tolVal_s=str2double(get(hObject,'String'));
handles.tol_rtheta(1,4)=tolVal_s;
assignin('base','tol_rtheta',handles.tol_rtheta);

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit42_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit43_Callback(hObject, eventdata, handles)
% hObject    handle to edit43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit43 as text
%        str2double(get(hObject,'String')) returns contents of edit43 as a double


% --- Executes during object creation, after setting all properties.
function edit43_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_2_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_3_Callback(hObject, eventdata, handles)
% New Subject
% hObject    handle to Untitled_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(handles.figure1)
TMS_LAB


% --------------------------------------------------------------------
function Untitled_4_Callback(hObject, eventdata, handles)
% Save
% hObject    handle to Untitled_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_5_Callback(hObject, eventdata, handles)
% Exit
% hObject    handle to Untitled_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(handles.figure1)


% --------------------------------------------------------------------
function Untitled_6_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_7_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%Add name to arrays
%1
handles.Subject_Information=evalin('base','Subject_Information');
handles.Subject_Information={handles.Subject_Information};
assignin('base','Subject_Information',handles.Subject_Information);
handles.Name1={'Subject_Name'};
assignin('base','Name1',handles.Name1)

%2
handles.Session_Date=evalin('base','Session_Date');
handles.Session_Date={handles.Session_Date};
assignin('base','Session_Date',handles.Session_Date);
handles.Name2={'Session_Date'};
assignin('base','Name2',handles.Name2)

%3
handles.Session_Number=evalin('base','Session_Number');
handles.Session_Number={handles.Session_Number};
assignin('base','Session_Number',handles.Session_Number);
handles.Name3={'Session_Number'};
assignin('base','Name3',handles.Name3)

%4
handles.no_next_iter=evalin('base','no_next_iter');
handles.no_next_iter={handles.no_next_iter};
assignin('base','no_next_iter',handles.no_next_iter);
handles.Name4={'Maximum_number_of_pulses'};
assignin('base','Name4',handles.Name4)

%5
handles.nxeq0=evalin('base','nxeq0');
handles.nxeq0={handles.nxeq0};
assignin('base','nxeq0',handles.nxeq0);
handles.Name5={'Number_Of_Baselinedata'};
assignin('base','Name5',handles.Name5);

%6
handles.y_var0=evalin('base','y_var0');
handles.y_var0=handles.y_var0';
assignin('base','y_var0',handles.y_var0);
handles.Name6={'Baselinedata'};
assignin('base','Name6',handles.Name6)

%7
handles.xid_max=evalin('base','xid_max');
handles.xid_max=handles.xid_max;
assignin('base','xid_max',handles.xid_max);
handles.Name7={'Maximum_TMS_Amplitude'};
assignin('base','Name7',handles.Name7)

%8
handles.xid_fim_baseline=evalin('base','xid_fim_baseline');
handles.xid_fim_baseline=(handles.xid_fim_baseline)';
assignin('base','xid_fim_baseline',handles.xid_fim_baseline);
handles.Name8={'TMS_Pulses'};
assignin('base','Name8',handles.Name8)

%9
handles.yid_fim_baseline=evalin('base','yid_fim_baseline');
handles.yid_fim_baseline=(handles.yid_fim_baseline)';
assignin('base','yid_fim_baseline',handles.yid_fim_baseline);
handles.Name9={'MEP_Amplitudes'};
assignin('base','Name9',handles.Name9)

%10
handles.theta_est=evalin('base','theta_est');
handles.Name10={'Parameters';'yl,yh,m,s'};
assignin('base','Name10',handles.Name10);

%11
handles.tol_rtheta=evalin('base','tol_rtheta');
handles.tol_rtheta=(handles.tol_rtheta)';
assignin('base','tol_rtheta',handles.tol_rtheta);
handles.Name11={'Convergence_Tolerancees';'yl,yh,m,s'};
assignin('base','Name11',handles.Name11)

%12
handles.iter_ation=evalin('base','iter_ation');
handles.Name12={'Number_of_sampling'};
assignin('base','Name12',handles.Name12)

%13
handles.Stopping_rule_satisfied=evalin('base','Stopping_rule_satisfied');
handles.Stopping_rule_satisfied={handles.Stopping_rule_satisfied};
assignin('base','Stopping_rule_satisfied',handles.Stopping_rule_satisfied);
handles.Name13={'Stopping_rule_satisfied'};
assignin('base','Name13',handles.Name13)

%Creat structure consisting of workspace arrays
handles.Session_Data=struct('Subject_Name',handles.Subject_Information,'Session_Number',handles.Session_Number,'Session_Date',handles.Session_Date,'Maximum_number_of_Pulses',handles.no_next_iter,'Number_Of_Baselinedata',handles.nxeq0,'Recorded_Baselinedata',handles.y_var0,'Maximum_TMS_Amplitude',handles.xid_max,'TMS_Amplitude',handles.xid_fim_baseline,'MEP_Amplitude',handles.yid_fim_baseline,'theta_estimation',handles.theta_est,'Convergence_Tolerances',handles.tol_rtheta,'Number_of_sampling',handles.iter_ation,'Stopping_rule_satisfied',handles.Stopping_rule_satisfied);
assignin('base','Session_Data',handles.Session_Data);

%Get path
[F P]=uiputfile('*.xlsx','Session_Data','Save Data as...');
filename=fullfile(P,F);

%Save each of arrays 
%1
xlswrite(filename,handles.Name1,1,'A1');
xlswrite(filename,handles.Subject_Information,1,'B1');
%2
xlswrite(filename,handles.Name2,2,'A1');
xlswrite(filename,handles.Session_Date,2,'B1');
%3
xlswrite(filename,handles.Name3,3,'A1');
xlswrite(filename,handles.Session_Number,3,'B1');
%4
xlswrite(filename,handles.Name4,4,'A1');
xlswrite(filename,handles.no_next_iter,4,'B1');
%5
xlswrite(filename,handles.Name5,5,'A1');
xlswrite(filename,handles.nxeq0,5,'B1');
%6
xlswrite(filename,handles.Name6,6,'A1');
xlswrite(filename,handles.y_var0,6,'B1');
%7
xlswrite(filename,handles.Name7,7,'A1');
xlswrite(filename,handles.xid_max,7,'B1');
%8
xlswrite(filename,handles.Name8,8,'A1');
xlswrite(filename,handles.xid_fim_baseline,8,'B1');
%9
xlswrite(filename,handles.Name9,9,'A1');
xlswrite(filename,handles.yid_fim_baseline,9,'B1');
%10
xlswrite(filename,handles.Name10,10,'A1');
xlswrite(filename,handles.theta_est,10,'B1');
%11
xlswrite(filename,handles.Name11,11,'A1');
xlswrite(filename,handles.tol_rtheta,11,'B1');
%12
xlswrite(filename,handles.Name12,12,'A1');
xlswrite(filename,handles.iter_ation,12,'B1');
xlswrite(filename,handles.Name13,12,'C1');
xlswrite(filename,handles.Stopping_rule_satisfied,12,'D1');

clc






% --------------------------------------------------------------------
function Untitled_8_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[F P]=uiputfile('Input-Output Curve.fig','Save Figure as...');
NewFig1=figure;
NewAx1=copyobj(handles.axes1,NewFig1);
set(NewAx1,'Units','normalized','Position',[0.08 0.1 0.85 0.75]);
hgsave(NewFig1,[P F]);
close(NewFig1);


% --------------------------------------------------------------------
function Untitled_9_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[F P]=uiputfile('Lower Plateaus Curve.fig','Save Figure as...');
NewFig2=figure;
NewAx2=copyobj(handles.axes2,NewFig2);
set(NewAx2,'Units','normalized','Position',[0.08 0.1 0.85 0.75]);
hgsave(NewFig2,[P F]);
close(NewFig2);


% --------------------------------------------------------------------
function Untitled_10_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[F P]=uiputfile('Higher Plateaus Curve.fig','Save Figure as...');
NewFig3=figure;
NewAx3=copyobj(handles.axes3,NewFig3);
set(NewAx3,'Units','normalized','Position',[0.08 0.1 0.85 0.75]);
hgsave(NewFig3,[P F]);
close(NewFig3);



% --------------------------------------------------------------------
function Untitled_11_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[F P]=uiputfile('Mid point Curve.fig','Save Figure as...');
NewFig4=figure;
NewAx4=copyobj(handles.axes4,NewFig4);
set(NewAx4,'Units','normalized','Position',[0.08 0.1 0.85 0.75]);
hgsave(NewFig4,[P F]);
close(NewFig4);


% --------------------------------------------------------------------
function Untitled_12_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[F P]=uiputfile('Slope Curve.fig','Save Figure as...');
NewFig5=figure;
NewAx5=copyobj(handles.axes5,NewFig5);
set(NewAx5,'Units','normalized','Position',[0.08 0.1 0.85 0.75]);
hgsave(NewFig5,[P F]);
close(NewFig5);
