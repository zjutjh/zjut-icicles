function varargout = trnsfm(varargin)
% TRNSFM M-file for trnsfm.fig
%      TRNSFM, by itself, creates a new TRNSFM or raises the existing
%      singleton*.
%
%      H = TRNSFM returns the handle to a new TRNSFM or the handle to
%      the existing singleton*.
%
%      TRNSFM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRNSFM.M with the given input arguments.
%
%      TRNSFM('Property','Value',...) creates a new TRNSFM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before trnsfm_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to trnsfm_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help trnsfm

% Last Modified by GUIDE v2.5 30-Sep-2013 23:18:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @trnsfm_OpeningFcn, ...
                   'gui_OutputFcn',  @trnsfm_OutputFcn, ...
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


% --- Executes just before trnsfm is made visible.
function trnsfm_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to trnsfm (see VARARGIN)

% Choose default command line output for trnsfm
handles.output = hObject;
Fs=8000;                       %采样频率，16000表示采样频率为16000Hz（可改为8000, 11025, 22050, 44100, 48000等）
Nbits=16;                       %量化位数，16为用16bits存储(可改为8，16，24)
Ch=1;                           %声道数（也可以改为2,即立体声）
R=audiorecorder(Fs,Nbits,Ch);   %创建一个保存音频信息的对象，它包含采样率，时间和录制的音频信息等等


%16为用16bits存储2为两通道即立体声（也可以改为1即单声道）
handles.R=R;
handles.Fs=Fs;
handles.Nbits=Nbits;

% Update handles structure
guidata(hObject, handles);


% UIWAIT makes trnsfm wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = trnsfm_OutputFcn(hObject, eventdata, handles) 
% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% Start to recording
record(handles.R);
guidata(hObject,handles);

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% Quit the program
close(gcf);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, ~, handles)
% Stop recording
stop(handles.R);
myvoice=getaudiodata(handles.R);
Fs=handles.Fs;
Nbits=handles.Nbits;
audiowrite('myspeech.wav',myvoice,Fs,'BitsPerSample',Nbits);
% audiowrite(myspeech,Fs,Nbits,'myspeech.mat');
handles.myspeech=myspeech;
guidata(hObject,handles);

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(~, ~, handles)
% Play the recording
para=get(handles.popupmenu1,'Value');
switch para
    case 1
        [speechdata,fs]=audioread('myspeech.wav');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
    case 2
        [speechdata,fs]=audioread('myspeech_sing.mat');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
    case 3
        [speechdata,fs]=audioread('myspeech_welcome.mat');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
end


axes(handles.axes1);

n=0:(length(speechdata)-1);
plot(n/fs,speechdata);
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
xlabel('\fontname{Time New Roman}\itt \rm(sec)');
ylabel('\fontname{Time New Roman}\itx\rm(\itt\rm)');
axis tight

% audioplayer(speechdata,fs);
sound(speechdata,fs);

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% Time Shifting and Echoes
para=get(handles.popupmenu1,'Value');
switch para
    case 1
        [speechdata,fs]=audioread('myspeech.wav');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
    case 2
        [speechdata,fs]=audioread('myspeech_sing.mat');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
    case 3
        [speechdata,fs]=audioread('myspeech_welcome.mat');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
end


N=length(speechdata);

n00= str2double(get(handles.edit2,'string'));
if isnan(n00)
    set(handles.edit2, 'String', 0);
    errordlg('Input must be a number','Error');
end
alpha10 = str2double(get(handles.edit3,'string'));
if isnan(alpha10)
    set(handles.edit3, 'String', 0);
    errordlg('Input must be a number','Error');
end
alpha20 = str2double(get(handles.edit4,'string'));
if isnan(alpha20)
    set(handles.edit4, 'String', 0);
    errordlg('Input must be a number','Error');
end

n0=n00/1000*fs;
alpha1=alpha10/100;
alpha2=alpha20/100;

x10=[speechdata;zeros(2*n0,1)];
x11=[zeros(n0,1);speechdata;zeros(n0,1)];
x12=[zeros(2*n0,1);speechdata];
for n=1:(N+2*n0)
    echoedspch(n)=x10(n)+alpha1*x11(n)+alpha2*x12(n);
end

axes(handles.axes3);

n=0:(length(echoedspch))-1;
plot(n/fs,echoedspch);
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
xlabel('\fontname{Time New Roman}\itt \rm(sec)');
ylabel('\fontname{Time New Roman}\itx\rm(\itt\rm) \rm+\it\alpha\rm_1\itx\rm(\itt-t\rm_0)+\it\alpha\rm_2\itx\rm(\itt\rm-2\itt\rm_0)');
axis tight

sound(echoedspch,fs);
% audioplayer(echoedspch,fs);



% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% Time reversal
para=get(handles.popupmenu1,'Value');
switch para
    case 1
        [speechdata,fs]=audioread('myspeech.wav');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
    case 2
        [speechdata,fs]=audioread('myspeech_sing.mat');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
    case 3
        [speechdata,fs]=audioread('myspeech_welcome.mat');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
end


N=length(speechdata);

for m=1:N                              %Reverse the speech signal
    reversedspeech(m)=speechdata(N-m+1);
end

axes(handles.axes2);

n=1-N:0;
plot(n/fs,reversedspeech);
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
xlabel('\fontname{Time New Roman}\itt \rm(sec)');
ylabel('\fontname{Time New Roman}\itx\rm(\it-t\rm)');
axis tight

% audioplayer(reversedspeech,fs);
sound(reversedspeech,fs);


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% Time scaling
para=get(handles.popupmenu1,'Value');
switch para
    case 1
        [speechdata,fs]=audioread('myspeech.wav');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
    case 2
        [speechdata,fs]=audioread('myspeech_sing.mat');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
    case 3
        [speechdata,fs]=audioread('myspeech_welcome.mat');    %得到以n*2列数字矩阵存储的刚录制的音频信号。
end

N=length(speechdata);

alpha=str2double(get(handles.edit5,'string'));

if isnan(alpha)
    set(handles.edit5, 'String',1);
    errordlg('Input must be a number','Error');
end

axes(handles.axes4);

n=0:N-1;
plot(n/fs/alpha,speechdata);
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
xlabel('\fontname{Time New Roman}\itt \rm(sec)');
ylabel('\fontname{Time New Roman}\itx\rm(\it\alphat\rm)');
axis tight

% audioplayer(speechdata,fs*alpha);
sound(speechdata,fs*alpha);

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit4_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
movegui(hObject,'center');


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
        set(handles.pushbutton1,'Visible','on')
        set(handles.pushbutton3,'Visible','on')
    case 2
        set(handles.pushbutton1,'Visible','off')
        set(handles.pushbutton3,'Visible','off')    
    case 3
        set(handles.pushbutton1,'Visible','off')
        set(handles.pushbutton3,'Visible','off')
end
% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
