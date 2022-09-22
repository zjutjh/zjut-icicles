function varargout = conv_animt(varargin)
% CONV_ANIMT M-file for conv_animt.fig
%      CONV_ANIMT, by itself, creates a new CONV_ANIMT or raises the existing
%      singleton*.
%
%      H = CONV_ANIMT returns the handle to a new CONV_ANIMT or the handle to
%      the existing singleton*.
%
%      CONV_ANIMT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONV_ANIMT.M with the given input arguments.
%
%      CONV_ANIMT('Property','Value',...) creates a new CONV_ANIMT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before conv_animt_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to conv_animt_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help conv_animt

% Last Modified by GUIDE v2.5 10-Jan-2013 23:19:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @conv_animt_OpeningFcn, ...
                   'gui_OutputFcn',  @conv_animt_OutputFcn, ...
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


% --- Executes just before conv_animt is made visible.
function conv_animt_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
movegui(hObject,'center');
guidata(hObject, handles);

% UIWAIT makes conv_animt wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = conv_animt_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
%%
x1=str2num(get(handles.edit1,'string'));
if isnan(x1)
    x1=0;
    set(handles.edit1,'String',[ 0 ]);
    errordlg('Input must be a number','Error');
end

x2=str2num(get(handles.edit2,'string'));
if isnan(x2)
    x2=0;
    set(handles.edit2,'String',[ 0 ]);
    errordlg('Input must be a number','Error');
end

N1=length(x1);
N2=length(x2);
x1_filled=[zeros(1,N2+1) x1 zeros(1,N2+1)];
x2_filled=[zeros(1,N2+1) x2 zeros(1,N1+1)];

x2_rev=fliplr(x2);

n=-N2-1:N1+N2;

%%
axes(handles.axes1);
stem(n,x1_filled,'filled','y','linewidth',2);
axis([min(n) max(n) (min([x1 0])-0.1)*1.2 (max([x1 0])+0.1)*1.2])
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
xlabel('\itm')
title('\itx\rm_1[\itm\rm]','color','w');

axes(handles.axes2);
stem(n,x2_filled,'.','r');
axis([min(n) max(n) (min([x2 0])-0.1)*1.2 (max([x2 0])+0.1)*1.2])
set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
xlabel('\itm')
title('\itx\rm_2[\itm\rm]','color','w');

for n0=1:N1+N2+3
    x2_rev_shift=[zeros(1,n0-1) x2_rev zeros(1,N1+N2-n0+3)];
     if n0==1
         result=0;
     else
         result=[result sum(x1_filled.*x2_rev_shift)];
     end
    y=[zeros(1,N2-1) result zeros(1,N1+N2-n0+3)];
    
    axes(handles.axes3);
    stem(n,x1_filled,'filled','y','linewidth',2);
    xlabel('\itm')
    set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);    
    hold on;
    stem(n,x2_rev_shift,'.','r','LineWidth',1);
    axis([min(n) max(n) (min([x1 x2 0])-0.1)*1.2 (max([x1 x2 0])+0.1)*1.2])
    legend_h=legend(handles.axes3,'x_1[m]','x_2[n-m]','Location',[0.63 0.37 0.08 0.08]);
    set(legend_h,'Textcolor','w')
    hold off;
        
    axes(handles.axes4);
    stem(n,y,'.','g');
    axis([min(n) max(n) (min([conv(x1,x2) 0])-0.1)*1.2 (max([conv(x1,x2) 0])+0.1)*1.2])
    set(gca,'color',[0 0 0],'XColor',[1 1 1],'YColor',[1 1 1]);
    xlabel('\itn')
    ylabel('\ity\rm[\itn\rm]')
    title(['\ity\rm[' num2str(n0-3) ']=' num2str(y(n0+N2-1))],'color','w');
    
    set(handles.text3,'string','Press ENTER key to see the next step ...')
    pause();
 
end
set(handles.text3,'string','That is the end.')

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
clear
close(gcf);
