function varargout = msceq(varargin)
%MSCEQ M-file for msceq.fig
%      MSCEQ, by itself, creates a new MSCEQ or raises the existing
%      singleton*.
%
%      H = MSCEQ returns the handle to a new MSCEQ or the handle to
%      the existing singleton*.
%
%      MSCEQ('Property','Value',...) creates a new MSCEQ using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to msceq_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      MSCEQ('CALLBACK') and MSCEQ('CALLBACK',hObject,...) call the
%      local function named CALLBACK in MSCEQ.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help msceq

% Last Modified by GUIDE v2.5 23-Sep-2011 02:05:32

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @msceq_OpeningFcn, ...
                   'gui_OutputFcn',  @msceq_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
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


% --- Executes just before msceq is made visible.
function msceq_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for msceq
handles.output = hObject;
movegui(hObject,'center');

Fs=16000;                       %采样频率，16000表示采样频率为16000Hz（可改为8000, 11025, 22050, 44100, 48000等）
Nbits=16;                       %量化位数，16为用16bits存储
Ch=1;                           %声道数（也可以改为2,即立体声）
R=audiorecorder(Fs,Nbits,Ch);   %创建一个保存音频信息的对象，它包含采样率，时间和录制的音频信息等等
[speechdata,fs]=audioread('mymusic.mat');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
N=length(speechdata);
whtns=wgn(1,length(speechdata),0);
t=linspace(0,N/441,N);
n1=sin(2*pi*90*t);
n2=sin(2*pi*80*t);
n3=sin(2*pi*100*t);
n4=sin(2*pi*72*t);
n5=sin(2*pi*68*t);
n6=sin(2*pi*74*t);
n7=sin(2*pi*65*t);
s=speechdata(:,1)+0.1*whtns'+0.05*(0.21*n1'+0.19*n2'+0.14*n3'+0.15*n4'+0.20*n5'+0.26*n6'+0.12*n7');

handles.noised=s;
handles.R=R;
handles.Fs=Fs;
handles.fs=fs;
handles.Nbits=Nbits;
handles.fm=20000;
handles.inputmax=max(abs(fft(speechdata(:,1))));


m=[1.6218 	1.6218 	1.4962 	1.0000 	0.5754 	0.6839 	1.0000 	1.7783 	2.1135 	2.3174 	2.3174 	2.3174];

n=128; % order of the system
f=[0 0.00317 0.00816 0.01451 0.02721 0.04535 0.13605 0.27211 0.54422 0.63492 0.72562 1];
% f is the frequencies points normalized by 22050Hz
hn=fir2(n,f,m);
[origin,fs]=audioread('mymusic.mat');
axes(handles.axes2)
fw=abs(fft(hn));
fltd=fftshift(fw)/max(fw);
freq=-fs/2:fs/(length(fltd)-1):fs/2;
plot(freq',fltd,'y')
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
axis([0 handles.fm -0.1 1.1])
xlabel('\itf \rm(Hz)')
title('The Gain of the filter','color','w');
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes msceq wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = msceq_OutputFcn(hObject, eventdata, handles)

varargout{1} = handles.output;


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% Quit the program
close(gcf);



% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)

% Process the music and play
val=get(handles.popupmenu1,'value');

switch val
    case 1
        m=[1.6218 	1.6218 	1.4962 	1.0000 	0.5754 	0.6839 	1.0000 	1.7783 	2.1135 	2.3174 	2.3174 	2.3174]; 
    case 2
        m=[1.9498 	1.9498 	1.9498 	1.9498 	1.4962 	1.0965 	0.7161 	0.5248 	0.4624 	0.4416 	0.4416 	0.4416]; 
    case 3
        m=[0.4842 	0.4842 	0.4842 	0.4842 	0.7161 	1.2023 	2.1135 	3.0200 	3.0200 	3.0200 	3.2734 	3.2734]; 
    case 4
        m=[1.0000 	1.0000 	1.0000 	1.0000 	1.0000 	0.0001 	0.0001 	0.0001 	0.0001 	0.0001 	0.0001 	0.0001]; 
    case 5
        m=[0.0001 	0.0001 	0.0001 	0.0001 	0.0001 	0.0001 	0.0001 	1.0000 	1.0000 	1.0000 	1.0000 	1.0000]; 
    case 6
        m=[0.0010 	0.0010 	0.0010 	1.0000 	1.0000 	1.0000 	1.0000 	0.0010 	0.0010 	0.0010 	0.0010 	0.0010]; 
end   
        
n=512; % order of the system
f=[0 0.00317 0.00816 0.01451 0.02721 0.04535 0.13605 0.27211 0.54422 0.63492 0.72562 1];
% f is the frequencies points normalized by 22050Hz

hn=fir2(n,f,m);        

[origin,fs]=audioread('mymusic.mat');% load the original audio signal

processed(:,1)=conv(origin(:,1),hn');% left channel audio processing by the system
processed(:,2)=conv(origin(:,2),hn');% right channel audio processing by the system

axes(handles.axes3)
fw=abs(fft(processed(:,1)));
inputmax=handles.inputmax;
fltd=fftshift(fw)/inputmax;
freq=-fs/2:fs/(length(fltd)-1):fs/2;
plot(freq',fltd)
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
axis([0 handles.fm -0.1 1.1])
xlabel('\itf \rm(Hz)')
title('The spectrm of filtered signal','color','w');

% audioplayer(processed,fs); 
sound(processed,fs);% play the processed audio signal

% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)

val=get(handles.popupmenu1,'value');

switch val
    case 1
        m=[1.6218 	1.6218 	1.4962 	1.0000 	0.5754 	0.6839 	1.0000 	1.7783 	2.1135 	2.3174 	2.3174 	2.3174]; 
    case 2
        m=[1.9498 	1.9498 	1.9498 	1.9498 	1.4962 	1.0965 	0.7161 	0.5248 	0.4624 	0.4416 	0.4416 	0.4416]; 
    case 3
        m=[0.4842 	0.4842 	0.4842 	0.4842 	0.7161 	1.2023 	2.1135 	3.0200 	3.0200 	3.0200 	3.2734 	3.2734]; 
    case 4
        m=[1.0000 	1.0000 	1.0000 	1.0000 	1.0000 	0.0001 	0.0001 	0.0001 	0.0001 	0.0001 	0.0001 	0.0001]; 
    case 5
        m=[0.0001 	0.0001 	0.0001 	0.0001 	0.0001 	0.0001 	0.0001 	1.0000 	1.0000 	1.0000 	1.0000 	1.0000]; 
    case 6
        m=[0.0010 	0.0010 	0.0010 	1.0000 	1.0000 	1.0000 	1.0000 	0.0010 	0.0010 	0.0010 	0.0010 	0.0010]; 
end   
        
n=128; % order of the system
f=[0 0.00317 0.00816 0.01451 0.02721 0.04535 0.13605 0.27211 0.54422 0.63492 0.72562 1];
% f is the frequencies points normalized by 22050Hz

hn=fir2(n,f,m);


[origin,fs]=audioread('mymusic.mat');
axes(handles.axes2)
fw=abs(fft(hn));
fltd=fftshift(fw)/max(fw);
freq=-fs/2:fs/(length(fltd)-1):fs/2;
plot(freq',fltd,'y')
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
axis([0 handles.fm -0.1 1.1])
xlabel('\itf \rm(Hz)')
title('The Gain of the filter','color','w');

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% Play the original record
[mymusic,fs]=audioread('mymusic.mat');    %得到以n*2列数字矩阵存储的刚录制的音频信号。

axes(handles.axes1)

fw=abs(fft(mymusic(:,1)));
inputmax=handles.inputmax;
fltd=fftshift(fw)/inputmax;
freq=-fs/2:fs/(length(fltd)-1):fs/2;
plot(freq',fltd,'g')
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
axis([0 handles.fm -0.1 1.1])
xlabel('\itf \rm(Hz)')
title('The spectrm of the original signal','color','w');

sound(mymusic,fs);


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

load lp_filter
s=handles.noised;
fs=handles.fs;
processed=conv(s,lp_filter');

axes(handles.axes2)
fw=abs(fft(lp_filter));
fltd2=fftshift(fw)/max(fw);
freq2=-fs/2:fs/(length(fltd2)-1):fs/2;
plot(freq2',fltd2,'y')
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
axis([0 handles.fm -0.1 1.1])
xlabel('\itf \rm(Hz)')
title('The Gain of the filter','color','w');

axes(handles.axes3)
fww=abs(fft(processed(:,1)));
fltd3=fftshift(fww)/max(fww);
freq3=-fs/2:fs/(length(fltd3)-1):fs/2;
plot(freq3',fltd3)
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
axis([0 handles.fm -0.1 1.1])
xlabel('\itf \rm(Hz)')
title('The spectrm of filtered signal','color','w');

sound(processed,fs);

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
s=handles.noised;
fs=handles.fs;

axes(handles.axes1)

fw=abs(fft(s(:,1)));
inputmax=max(fw);
fltd=fftshift(fw)/inputmax;
freq=-fs/2:fs/(length(fltd)-1):fs/2;
plot(freq',fltd,'g')
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
axis([0 handles.fs/2 -0.1 1.1])
xlabel('\itf \rm(Hz)')
title('The spectrm of the original signal','color','w');

sound(s,fs);
