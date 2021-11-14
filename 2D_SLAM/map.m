function varargout = map(varargin)
% MAP MATLAB code for map.fig
%      MAP, by itself, creates a new MAP or raises the existing
%      singleton*.
%
%      H = MAP returns the handle to a new MAP or the handle to
%      the existing singleton*.
%
%      MAP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAP.M with the given input arguments.
%
%      MAP('Property','Value',...) creates a new MAP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before map_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to map_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help map

% Last Modified by GUIDE v2.5 28-Dec-2020 21:24:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @map_OpeningFcn, ...
                   'gui_OutputFcn',  @map_OutputFcn, ...
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


% --- Executes just before map is made visible.
function map_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to map (see VARARGIN)

% Choose default command line output for map
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes map wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = map_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear
t_client=tcpip('192.168.43.19',9000,'NetworkRole','client');
t_client.OutputBuffersize=100000;
fopen(t_client);%与一个服务器建立连接，直到建立完成返回，否则报错。
txt_send='000'; %发送的文本数据
% pause(1);
fprintf(t_client,txt_send);%发送文本数据
fclose(t_client);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% clear
t_client=tcpip('192.168.43.19',9000,'NetworkRole','client');
t_client.OutputBuffersize=100000;
fopen(t_client);%与一个服务器建立连接，直到建立完成返回，否则报错。
txt_send='111'; %发送的文本数据
% pause(1);
fprintf(t_client,txt_send);%发送文本数据
fclose(t_client);
% pause(0.05);

% en=1;
% i=1;
% t_client=tcpip('192.168.43.19',9000,'NetworkRole','client');
% t_client.InputBuffersize=100000;
% fopen(t_client);
% while(1)
%     if  t_client.BytesAvailable>0
%         data_recv=str2num(char(fread(t_client,get(t_client,'BytesAvailable'),'char')'));%从缓冲区读取数字数据
%         if data_recv ~=0
%             H=size(data_recv);
%             M(i:i+H(1)-1,:)=data_recv;
%             i=i+H(1);
%             figure(5)
%             plot(M(:,1),M(:,2),'k-');
%             drawnow;
%             axis([-6000 1000 -2500 2000]);
%             axis manual
%             title("Map");
%             xlabel("X/mm");
%             ylabel("Y/mm");
%         end
%     end
%     if en==0
%         break
%     end
% end

% fclose(t_client);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear
t_client=tcpip('192.168.43.19',9000,'NetworkRole','client');
t_client.OutputBuffersize=100000;
fopen(t_client);%与一个服务器建立连接，直到建立完成返回，否则报错。
txt_send='999'; %发送的文本数据
% pause(1);
fprintf(t_client,txt_send);%发送文本数据
en=0;
fclose(t_client);


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1
