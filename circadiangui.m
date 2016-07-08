function varargout = circadiangui(varargin)
% CIRCADIANGUI MATLAB code for circadiangui.fig
%      CIRCADIANGUI, by itself, creates a new CIRCADIANGUI or raises the existing
%      singleton*.
%
%      H = CIRCADIANGUI returns the handle to a new CIRCADIANGUI or the handle to
%      the existing singleton*.
%
%      CIRCADIANGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CIRCADIANGUI.M with the given input arguments.
%
%      CIRCADIANGUI('Property','Value',...) creates a new CIRCADIANGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before circadiangui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to circadiangui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help circadiangui

% Last Modified by GUIDE v2.5 04-Jun-2016 13:55:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @circadiangui_OpeningFcn, ...
                   'gui_OutputFcn',  @circadiangui_OutputFcn, ...
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


% --- Executes just before circadiangui is made visible.
function circadiangui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to circadiangui (see VARARGIN)

% Choose default command line output for circadiangui
handles.output = hObject;
set(handles.threshold_slider,'value',40);

%% Query available camera and modes
imaqreset
c=imaqhwinfo;

for i=1:length(c.InstalledAdaptors)
    camInfo=imaqhwinfo(c.InstalledAdaptors{i});
    if ~isempty(camInfo.DeviceIDs)
        adaptor=i;
    end
end
camInfo=imaqhwinfo(c.InstalledAdaptors{adaptor});

if ~isempty(camInfo.DeviceInfo.SupportedFormats);
set(handles.Cam_popupmenu,'String',camInfo.DeviceInfo.SupportedFormats);
set(handles.Cam_popupmenu,'Value',1);
handles.Cam_mode=camInfo.DeviceInfo.SupportedFormats(1);
else
set(handles.Cam_popupmenu,'String','Camera not detected');
end
handles.camInfo=camInfo;


%% Initialize teensy for motor and light board control

%Close and delete any open serial objects
if ~isempty(instrfindall)
fclose(instrfindall);           % Make sure that the COM port is closed
delete(instrfindall);           % Delete any serial objects in memory
end

% Attempt handshake with light panel teensy
[handles.teensy_port,ports]=identifyMicrocontrollers;

% Update GUI menus with port names
set(handles.microcontroller_popupmenu,'string',handles.teensy_port);

% Initialize light panel at default values
IR_intensity=str2num(get(handles.edit_IR_intensity,'string'));
White_intensity=str2num(get(handles.edit_White_intensity,'string'));

% Convert intensity percentage to uint8 PWM value 0-255
IR_intensity=uint8((IR_intensity/100)*255);
handles.White_intensity=uint8((White_intensity/100)*255);

% Write values to microcontroller
writeInfraredWhitePanel(handles.teensy_port,0,IR_intensity);
writeInfraredWhitePanel(handles.teensy_port,1,handles.White_intensity);

%% Initialize circadian parameters
handles.lights_ON=get(handles.edit_lightsON,'String');
handles.lights_OFF=get(handles.edit_lightsOFF,'string');
handles.pulse_frequency=str2num(get(handles.edit_pulse_frequency,'string'));
handles.pulse_number=str2num(get(handles.edit_pulse_number,'string'));
handles.pulse_amplitude=str2num(get(handles.edit_pulse_amp,'string'));
handles.pulse_interval=str2num(get(handles.edit_pulse_interval,'string'));
handles.ref_stack_size=str2num(get(handles.edit_ref_stack_size,'String'));
handles.ref_freq=str2num(get(handles.edit_ref_freq,'String'));
handles.exp_duration=str2num(get(handles.edit_exp_duration,'String'));
handles.cam_gain=str2num(get(handles.edit_gain,'String'));
handles.cam_exposure=str2num(get(handles.edit_exposure,'String'));
handles.tracking_thresh=get(handles.threshold_slider,'Value');

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes circadiangui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = circadiangui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in microcontroller_popupmenu.
function microcontroller_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to microcontroller_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns microcontroller_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from microcontroller_popupmenu


% --- Executes during object creation, after setting all properties.
function microcontroller_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to microcontroller_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Cam_popupmenu.
function Cam_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to Cam_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
strCell=get(handles.Cam_popupmenu,'string');
handles.Cam_mode=strCell(get(handles.Cam_popupmenu,'Value'));
guidata(hObject, handles);


% Hints: contents = cellstr(get(hObject,'String')) returns Cam_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Cam_popupmenu


% --- Executes during object creation, after setting all properties.
function Cam_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cam_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_IR_intensity_Callback(hObject, eventdata, handles)
% hObject    handle to edit_IR_intensity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Initialize light panel at default values
IR_intensity=str2num(get(handles.edit_IR_intensity,'string'));

% Convert intensity percentage to uint8 PWM value 0-255
IR_intensity=uint8((IR_intensity/100)*255);

writeInfraredWhitePanel(handles.teensy_port,1,IR_intensity);



% --- Executes during object creation, after setting all properties.
function edit_IR_intensity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_IR_intensity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_White_intensity_Callback(hObject, eventdata, handles)
% hObject    handle to edit_White_intensity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
White_intensity=str2num(get(handles.edit_White_intensity,'string'));

% Convert intensity percentage to uint8 PWM value 0-255
handles.White_intensity=uint8((White_intensity/100)*255);
writeInfraredWhitePanel(handles.teensy_port,0,handles.White_intensity);

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_White_intensity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_White_intensity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_lightsON_Callback(hObject, eventdata, handles)
% hObject    handle to edit_lightsON (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
tString=get(handles.edit_lightsON,'String');
divider=find(tString==':');
hr=str2num(tString(1:divider-1));
min=str2num(tString(divider+1))*10+str2num(tString(divider+2));
handles.lights_ON=[hr min];
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_lightsON_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_lightsON (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_lightsOFF_Callback(hObject, eventdata, handles)
% hObject    handle to edit_lightsOFF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
tString=get(handles.edit_lightsOFF,'String');
divider=find(tString==':');
hr=str2num(tString(1:divider-1));
min=str2num(tString(divider+1))*10+str2num(tString(divider+2));
handles.lights_OFF=[hr min];
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_lightsOFF_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_lightsOFF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_exposure_Callback(hObject, eventdata, handles)
% hObject    handle to edit_exposure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.cam_exposure=str2num(get(handles.edit_exposure,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_exposure_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_exposure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_gain_Callback(hObject, eventdata, handles)
% hObject    handle to edit_gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.cam_gain=str2num(get(handles.edit_gain,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_gain_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pulse_frequency_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pulse_frequency (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.pulse_frequency=str2num(get(handles.edit_pulse_frequency,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_pulse_frequency_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pulse_frequency (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pulse_number_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pulse_number (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.pulse_number=str2num(get(handles.edit_pulse_number,'string'));

% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_pulse_number_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pulse_number (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pulse_amp_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pulse_amp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.pulse_amplitude=str2num(get(handles.edit_pulse_amp,'string'));

% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_pulse_amp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pulse_amp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pulse_interval_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pulse_interval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.pulse_interval=str2num(get(handles.edit_pulse_interval,'string'));

% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_pulse_interval_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pulse_interval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in save_path_button1.
function save_path_button1_Callback(hObject, eventdata, handles)
% hObject    handle to save_path_button1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[fpath] = uigetdir('E:\Decathlon Raw Data','Select a save destination');
handles.fpath=fpath;
set(handles.save_path,'String',fpath);
guidata(hObject,handles);



function save_path_Callback(hObject, eventdata, handles)
% hObject    handle to save_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of save_path as text
%        str2double(get(hObject,'String')) returns contents of save_path as a double


% --- Executes during object creation, after setting all properties.
function save_path_CreateFcn(hObject, eventdata, handles)
% hObject    handle to save_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Cam_confirm_pushbutton.
function Cam_confirm_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Cam_confirm_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
imaqreset;
pause(0.02);
handles.vid=initializeCamera(handles.camInfo.AdaptorName,handles.camInfo.DeviceIDs{1},handles.Cam_mode{:});
pause(0.1);
im=peekdata(handles.vid,1);
imshow(im);
guidata(hObject, handles);


% --- Executes on button press in Cam_preview_pushbutton.
function Cam_preview_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Cam_preview_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

stop=get(handles.Cam_preview_pushbutton,'Value');
if stop==1
    while stop==1
        imagedata=peekdata(handles.vid,1);
        pause(0.005);
        imshow(imagedata(:,:,2));
        stop=get(handles.Cam_preview_pushbutton,'Value');
    end
end


% --- Executes on button press in Cam_stopPreview_pushbutton.
function Cam_stopPreview_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Cam_stopPreview_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.Cam_preview_pushbutton,'Value',0);
set(handles.Cam_stopPreview_pushbutton,'Value',0);
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function labels_uitable_CreateFcn(hObject, eventdata, handles)
% hObject    handle to labels_uitable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
data=cell(5,8);
data(:)={''};
set(hObject, 'Data', data);
handles.labels=data;
guidata(hObject, handles);


% --- Executes when entered data in editable cell(s) in labels_uitable.
function labels_uitable_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to labels_uitable (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
handles.labels{eventdata.Indices(1), eventdata.Indices(2)} = {''};
handles.labels{eventdata.Indices(1), eventdata.Indices(2)} = eventdata.NewData;
guidata(hObject, handles);


% --- Executes on button press in motor_test_pushbutton.
function motor_test_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to motor_test_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

writeVibrationalMotors(handles.teensy_port,6,handles.pulse_frequency,handles.pulse_interval,...
    handles.pulse_number,handles.pulse_amplitude);



function edit_ref_stack_size_Callback(hObject, eventdata, handles)
% hObject    handle to edit_ref_stack_size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.ref_stack_size=str2num(get(handles.edit_ref_stack_size,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_ref_stack_size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_ref_stack_size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_ref_freq_Callback(hObject, eventdata, handles)
% hObject    handle to edit_ref_freq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.ref_freq=str2num(get(handles.edit_ref_freq,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_ref_freq_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_ref_freq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_exp_duration_Callback(hObject, eventdata, handles)
% hObject    handle to edit_exp_duration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.exp_duration=str2num(get(handles.edit_exp_duration,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_exp_duration_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_exp_duration (see GCBO)
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
if isfield(handles, 'fpath') == 0 
    errordlg('Please specify Save Location')
elseif isfield(handles, 'vid') == 0
    errordlg('Please confirm camera settings')
else
    circadianTracker;
    disp('all done')
end


% --- Executes on slider movement.
function threshold_slider_Callback(hObject, eventdata, handles)
% hObject    handle to threshold_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.tracking_thresh=get(handles.threshold_slider,'Value');
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function threshold_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to threshold_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in accept_thresh_pushbutton.
function accept_thresh_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to accept_thresh_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.accept_thresh_pushbutton,'value',1);
guidata(hObject, handles);



function edit_frame_rate_Callback(hObject, eventdata, handles)
% hObject    handle to edit_frame_rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_frame_rate as text
%        str2double(get(hObject,'String')) returns contents of edit_frame_rate as a double


% --- Executes during object creation, after setting all properties.
function edit_frame_rate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_frame_rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
