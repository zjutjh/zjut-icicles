function varargout = signal_recons(varargin)
% SIGNAL_RECONS M-file for signal_recons.fig
%      SIGNAL_RECONS, by itself, creates a new SIGNAL_RECONS or raises the existing
%      singleton*.
%
%      H = SIGNAL_RECONS returns the handle to a new SIGNAL_RECONS or the handle to
%      the existing singleton*.
%
%      SIGNAL_RECONS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIGNAL_RECONS.M with the given input arguments.
%
%      SIGNAL_RECONS('Property','Value',...) creates a new SIGNAL_RECONS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before signal_recons_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to signal_recons_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help signal_recons

% Last Modified by GUIDE v2.5 13-Oct-2011 01:55:49

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @signal_recons_OpeningFcn, ...
                   'gui_OutputFcn',  @signal_recons_OutputFcn, ...
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


% --- Executes just before signal_recons is made visible.
function signal_recons_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to signal_recons (see VARARGIN)

% Choose default command line output for signal_recons
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes signal_recons wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = signal_recons_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
para=get(hObject,'Value');

switch para
    case 1
        set(handles.text7,'string','The cutoff frequency of the LPF:')
%        set(handles.text9,'string','(0-5fs) Hz')
        set(handles.text15,'string','')
        set(handles.text16,'string','')
        set(handles.edit10,'Visible','off')
        
    case 2
        set(handles.text7,'string','The cutoff frequency of the HPF:')
%        set(handles.text9,'string','(0-5fs) Hz')
        set(handles.text15,'string','')
        set(handles.text16,'string','')
        set(handles.edit10,'Visible','off')
        
    case 3
        set(handles.text7,'string','The central frequency of the BPF:')
%        set(handles.text9,'string','(0-5fs) Hz')
        set(handles.text15,'string','The passband width of the BPF:')
        set(handles.text16,'string','Hz')
        set(handles.edit10,'Visible','on')
        
    case 4
        set(handles.text7,'string','The central frequency of the BSF:')
%        set(handles.text9,'string','(0-5fs) Hz')
        set(handles.text15,'string','The stopband width of the BSF:')
        set(handles.text16,'string','Hz')
        set(handles.edit10,'Visible','on')

end

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


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



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%%%%%%%%% Inputs:
  %%% V_m = [ A1 A2 ... AN];   --- analog signal amplitudes 
  %%%% V_f= [ f1 f2 ... fN];   --- analog signal frequency
  %%%% V_p=[theta_1 theta_2 .... theta_N]  --- analog signal phase
  %%%% f_s   --- sampling frequency
  %%%% f_c   --- the cutoff frequency of the filter in the range between 0
  %%%%       --- and 1
  %%%% para  --- to choose the filter type; para=0, corresponding to the
  %%%%           low-pass filter; para=1, corresponding the high pass filter.
  
  %%%%%%%%%%%%%%%  Suggestion
   %%%% 1) you can change the amplitude V_m, the frequency V_f and the phase
   %%%%  V_p of the signal;
   %%%%% 2) you can change the sampling frequency f_s to observe the effect
   %%%%% to the signal reconstruction
   %%%%% 3) you can change the ccutoff frequency of the filter
 
%V_m = str2num(get(handles.edit1,'String'));
%V_f = str2num(get(handles.edit2,'String'));
%V_p = str2num(get(handles.edit3,'String'));
a1=str2double(get(handles.edit1,'String'));
a2=str2double(get(handles.edit2,'String'));
f1=str2double(get(handles.edit7,'String'));
f2=str2double(get(handles.edit9,'String'));
fai1=str2num(get(handles.edit8,'String'));
fai2=str2num(get(handles.edit3,'String'));

f_s = str2double(get(handles.edit4,'String'));
N = str2double(get(handles.edit5,'String'));
para=get(handles.popupmenu1,'Value');

switch para
    case 1
        fc= str2double(get(handles.edit6,'String'));
        fs=10*f_s;
        wc=2*fc/fs;
        b=fir1(N,wc,'low');
    case 2
        fc=str2double(get(handles.edit6,'String'));
        fs=10*f_s;
        wc=2*fc/fs;
        b=fir1(N,wc,'high');
    case 3
        fc=str2double(get(handles.edit6,'String'));
        fw=str2double(get(handles.edit10,'String'));
        fs=10*f_s;
        w1=2*(fc-fw/2)/fs;
        w2=2*(fc+fw/2)/fs;
        b=fir1(N,[w1 w2]);
    case 4
        fc=str2double(get(handles.edit6,'String'));
        fw=str2double(get(handles.edit10,'String'));
        fs=10*f_s;
        w1=2*(fc-fw/2)/fs;
        w2=2*(fc+fw/2)/fs;
        b=fir1(N,[w1 w2],'stop');
   
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
f_a=f_s*10;  
T_a=1/f_a;
Time=1;
Num=Time*f_a; 
Num=fix(Num+1);

t_a=0:1:Num-1;  
t_a=t_a*T_a;    % the anolog signal time array

%%%%%%%%% To generate "analog" signal
% [n1,n2]=size(V_m);  N_p=n1*n2;
%  x_a=t_a*0;            %  the analog signal of duration Time seconds
%for k=1:N_p
%    pp=V_m(k); qq=V_p(k);
%    x_a = x_a + pp*cos(2*pi*V_f(k)*t_a-qq);
%end

x_a=a1*sin(2*pi*f1*t_a+fai1)+a2*sin(2*pi*f2*t_a+fai2);

axes(handles.axes1)
plot(t_a,x_a);
hold on;
% %%%%%%%%% To generate discret-time signal using f_s

 Num=Time*f_s; Num=fix(Num+1);
t_d=0:1:Num-1;  t_d=t_d/f_s;    % time array
t_stop=(Num-1)/f_s;
% [n1,n2]=size(V_m);  N_p=n1*n2;
%  x=t_d*0;         %  the analog signal of duration Time seconds
%for k=1:N_p
%    pp=V_m(k); qq=V_p(k);
%    x = x + pp*cos(2*pi*V_f(k)*t_d-qq);
%end

x=a1*sin(2*pi*f1*t_d+fai1)+a2*sin(2*pi*f2*t_d+fai2);

stem(t_d,x)
title('The analog and sampled signal');
hold off
%%%%%%%%%%%%%%%%%%%%%% ideal signal reconstruction


t_sampling=1/f_s;
wc=2*pi*f_s/2;
n=0;
t_ideal=0:t_sampling/10:t_stop;
for t=0:t_sampling/10:t_stop;
     x1=0;
for i=1:Num
    m=t-(i-1)*t_sampling;
    if m==0;
        x1=x(i)/t_sampling;
    else
        x1=x1+x(i)*sin(wc*m)/(pi*m);
    end
end
n=n+1;
x11(n)=x1*t_sampling;
end

%%%%%%%%  zero-order reconstruction
% obtain the zero-order window
t2=-t_sampling:0.001:t_sampling;
h11=[];
m=1;
for t=-t_sampling:0.001:t_sampling
% if t<=t_sampling/2&&t>=-t_sampling/2
if t<=t_sampling&&t>=0
    h1=1;
else
    h1=0;
end
 h11(m)=h1;
    m=m+1;
end

% zero-order reconstruction from the discrete signal
t_zero=0:t_sampling/10:t_stop;
L=length(t_zero); 
x22=zeros(1,L);
for i=1:Num-1
    for j=i+(i-1)*9:i+(i-1)*9+9
      x2(j)=x(i);  
    end
end
x2(L)=x(Num-1);

%%%%%%%%%%%%% to plot the original signal and the reconstruted signal
axes(handles.axes2)
plot(t_a,x_a,'r','linewidth',1);
hold on;
plot(t_ideal,x11,'k','LineWidth',1);   % ideal reconstruction
title('The analog (red) and ideal(black) reconstructed signal');
hold off

axes(handles.axes3)
plot(t_a,x_a,'r','linewidth',1);
hold on;
stairs(t_d,x,'k','linewidth',1);     % zero-order reconstrution
% plot(t_zero,x2,'k','linewidth',1); 
title('The analog and zero-order reconstructed signal');
hold off

axes(handles.axes4)
plot(t_a,x_a,'r','linewidth',1);
hold on;
plot(t_d,x,'k','linewidth',1);     % first-order reconstrution
title('The analog and first-order reconstructed signal');
hold off
%%%%%%%%%%%%%%%%%  low-pass filter


x2_filter=filter(b,1,x2);
axes(handles.axes5)
plot(t_a,x_a,'g','linewidth',1);
hold on;
plot(t_zero,x2,'r','linewidth',1); 
hold on;
plot(t_zero,x2_filter,'k','linewidth',1); 
hold off;
title('The zero-order (red), original (green) and filtered reconstructed (black) signal');




% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(gcf);


function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
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
