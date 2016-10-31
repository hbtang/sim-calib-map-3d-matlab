function varargout = gui_lyj(varargin)
% GRAPHIC MATLAB code for graphic.fig
%      GRAPHIC, by itself, creates a new GRAPHIC or raises the existing
%      singleton*.
%
%      H = GRAPHIC returns the handle to a new GRAPHIC or the handle to
%      the existing singleton*.
%
%      GRAPHIC('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GRAPHIC.M with the given input arguments.
%
%      GRAPHIC('Property','Value',...) creates a new GRAPHIC or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before graphic_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to graphic_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help graphic

% Last Modified by GUIDE v2.5 30-Sep-2016 14:43:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @graphic_OpeningFcn, ...
                   'gui_OutputFcn',  @graphic_OutputFcn, ...
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


% --- Executes just before graphic is made visible.
function graphic_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to graphic (see VARARGIN)

javaFrame = get(hObject,'JavaFrame');
javaFrame.setFigureIcon(javax.swing.ImageIcon('splash.png'));

handles.filename = [pwd '\data'];
set(handles.edit_filename,'String',handles.filename);
set(handles.text_status,'String','待命中');

ntype = exist('default.mat');
if ntype == 2
    load('default.mat');
    handles.suffix = suffix;
    handles.markersize = markersize;
    handles.pt1_id = pt1_id;
    handles.pt1_x = pt1_x;
    handles.pt1_y = pt1_y;
    handles.pt2_id = pt2_id;
    handles.pt2_x = pt2_x;
    handles.pt2_y = pt2_y;
    handles.loopcount = loopcount; 
else
    handles.suffix = 'r4_002';
    handles.markersize = 59;
    handles.pt1_id = 60;
    handles.pt1_x = 2340;
    handles.pt1_y = -3550;
    handles.pt2_id = 49;
    handles.pt2_x = 8950;
    handles.pt2_y = -3550;
    handles.loopcount = 20; 
end

set(handles.edit_suffix,'String',handles.suffix);
set(handles.edit_markersize,'String',num2str(handles.markersize));
set(handles.edit_pt1_id,'String',num2str(handles.pt1_id));
set(handles.edit_pt1_x,'String',num2str(handles.pt1_x));
set(handles.edit_pt1_y,'String',num2str(handles.pt1_y));
set(handles.edit_pt2_id,'String',num2str(handles.pt2_id));
set(handles.edit_pt2_x,'String',num2str(handles.pt2_x));
set(handles.edit_pt2_y,'String',num2str(handles.pt2_y));
set(handles.edit_loopcount,'String',num2str(handles.loopcount));

% Choose default command line output for graphic
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes graphic wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = graphic_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in openfolder.
function openfolder_Callback(hObject, eventdata, handles)
% hObject    handle to openfolder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
str = uigetdir(handles.filename);
if str ~= 0
    handles.filename = str;
    set(handles.edit_filename,'String',handles.filename);
end
 guidata(hObject, handles);

function edit_filename_Callback(hObject, eventdata, handles)
% hObject    handle to edit_filename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_filename as text
%        str2double(get(hObject,'String')) returns contents of edit_filename as a double
str = get(hObject,'String');
type = exist(str);
if type==7
    handles.filename = str;
else
    set(handles.edit_filename,'String',handles.filename);
    errordlg('File not found','File Error');
end
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_filename_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_filename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_calib.
function btn_calib_Callback(hObject, eventdata, handles)
% hObject    handle to btn_calib (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if exist([handles.filename '/thb_local_record_' handles.suffix '/record'],'file') == 7
    set(handles.text_status,'String','开始读取文件...请耐心等待');
    drawnow
    [mark, markNum, pt_corner_mark, odo] = init(handles.filename,handles.suffix,handles.markersize);
    set(handles.text_status,'String','读取文件完毕！开始计算相机外参初始值...');
    drawnow
    %% calculate norm vector of the ground plane
    vec_ground = cal_ground(mark, markNum, pt_corner_mark);

    %% modify 3D coordinates into 2D plane
    [T_cam3d_cam2d, mark] = proj_2_ground(mark,markNum,vec_ground);

    %% calibration in 2d plane
    [T_b_c] = calib_cam_extrinsic(mark,markNum,odo);
    set(handles.text_status,'String','计算相机外参初始值完毕！开始SLAM数据读取...');
    drawnow
    % initial guess for VisionNav robot system
    disp('initial guess for VisionNav robot system');
    disp('******************************************************')
    disp('T_b_c:')
    disp(T_b_c)
    disp('******************************************************')
    disp('T_cam3d_cam2d')
    disp(T_cam3d_cam2d)
    disp('******************************************************')

    %read slam data, throw some odo data
    [ odo_raw, mark ] = SLAM_ReadData(odo, mark);

    set(handles.text_status,'String','SLAM数据读取完毕！开始SLAM初始化...');
    drawnow

    sz_odo= numel(odo_raw.stamp);
    sz_mark = numel(mark.stamp);

    ref.id1 = handles.pt1_id; ref.x1 = handles.pt1_x; ref.y1 = handles.pt1_y;
    ref.id2 = handles.pt2_id; ref.x2 = handles.pt2_x; ref.y2 = handles.pt2_y;

    grid on;
    hold on;
    set(gca, 'DataAspectRatio',[1 1 1]);
    set(gca, 'fontsize', 10);

    %% distort odometry
    k_theta = 1;
    k_dist = 1;
    %T_b_c(1:2,3) = [200;0];
    %refineodo by change angle and dis by a const
    [ odo_refine ] = Proc_RefineOdo( odo_raw, k_dist, k_theta );
    rec_param = [k_dist, k_theta, T_b_c(1,3), T_b_c(2,3)];

    %% slam init
    disp('Intializing ...');

    [ Alpha, Omega, Xi, hash_Alpha_odoIdx, hash_Alpha_markIdx ] = SLAM_Init( odo_refine, mark, T_b_c );
    rec_map = cell(0,0);
    num_loop = handles.loopcount;

    disp('Initialization done.');
    set(handles.text_status,'String','SLAM初始化结束，开始标定及建图...');
    drawnow
    disp('Calibration and Mapping...');
    for i = 1:num_loop
        %% start clock
        t1 = clock();

        for j = 1:3
            %% reset optimization problem
            [ Omega, Xi, hash_mark_odo ] = SLAM_ResetOptFun( Alpha, hash_Alpha_odoIdx, hash_Alpha_markIdx,...
                odo_refine, mark, T_b_c, sz_odo, sz_mark );

            %% solve graph SLAM
            Alpha = Omega\Xi;
        end

        %% refine parameters: camera extrinsics and odometry param
        % need to modify the referece mark info ...
        [ T_b_c, k_dist, k_theta, Alpha, odo_refine] = Proc_RefineParam( Alpha, odo_raw, k_dist, k_theta, sz_odo, mark, sz_mark, T_b_c, ...
            hash_Alpha_odoIdx, hash_Alpha_markIdx, ref );

        rec_param_tmp = [k_dist, k_theta, T_b_c(1,3), T_b_c(2,3)];
        rec_param = [rec_param; rec_param_tmp];

        %% draw results
        % need to modify the referece mark info ...
        map_tmp = SLAM_ShowRes(Alpha, hash_mark_odo, sz_odo, sz_mark, hash_Alpha_odoIdx, hash_Alpha_markIdx, ref);
        rec_map{i,1} = map_tmp;
        pause(0.1);

        %% end clock
        t2 = clock();
        disp(['Loop ', num2str(i), ', using ',num2str(etime(t2,t1)), ' sec.',...
            'k_dist:', num2str(k_dist), ', k_theta:', num2str(k_theta), ...
            ', x_b_c:', num2str(T_b_c(1,3)), ', y_b_c:', num2str(T_b_c(2,3))]);
        str = ['Loop ', num2str(i), ', using ',num2str(etime(t2,t1)), ' sec.',...
            'k_dist:', num2str(k_dist), ', k_theta:', num2str(k_theta), ...
            ', x_b_c:', num2str(T_b_c(1,3)), ', y_b_c:', num2str(T_b_c(2,3))];
        set(handles.text_status,'String',str);
        drawnow
    end

    vec_id = hash_Alpha_markIdx(:,2);
    mat_map = full(map_tmp);
    [~,seq_ord] = sort(vec_id);

    vec_id_ord = zeros(size(vec_id));
    mat_map_ord = zeros(size(mat_map));

    for i = 1:numel(vec_id_ord)
        row = seq_ord(i);
        vec_id_ord(i) = vec_id(row);
        mat_map_ord(i,:) = mat_map(row,:);    
    end

    mkdir([handles.filename '\..\calibration_results']);
    outputFilePath = [handles.filename '\..\calibration_results\' 'calibmap_' handles.suffix '.yml'];
    outputFileId = fopen(outputFilePath,'w');
    fprintf(outputFileId,'%%YAML:1.0\n');
    writeYmlMatrix( outputFileId, T_cam3d_cam2d, 'T_cam3d_cam2d', 'f');
    writeYmlMatrix( outputFileId, T_b_c, 'T_base_cam', 'f');
    % writeYmlMatrix( outputFileId, vec_id_ord, 'vec_id', 'd');
    % writeYmlMatrix( outputFileId, mat_map_ord, 'map_matrix', 'f');
    writeYmlMatrix( outputFileId, [vec_id_ord mat_map_ord], 'map_id_matrix', 'map');

    fclose(outputFileId);
    set(handles.text_status,'String','标定成功！已完成文件输出！');
    guidata(hObject,handles);

    pt1_id = handles.pt1_id;
    pt1_x = handles.pt1_x;
    pt1_y = handles.pt1_y;

    pt2_id = handles.pt2_id;
    pt2_x = handles.pt2_x;
    pt2_y = handles.pt2_y;

    suffix = handles.suffix;
    markersize = handles.markersize;
    loopcount = handles.loopcount;

    save('default.mat','pt1_id','pt2_id','pt1_x','pt1_y','pt2_x','pt2_y','suffix','markersize','loopcount');
else
    errordlg('Cannot find record folder','Input Error');
end

function edit_suffix_Callback(hObject, eventdata, handles)
% hObject    handle to edit_suffix (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_suffix as text
%        str2double(get(hObject,'String')) returns contents of edit_suffix as a double
handles.suffix = get(hObject,'String');
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_suffix_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_suffix (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_markersize_Callback(hObject, eventdata, handles)
% hObject    handle to edit_markersize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_markersize as text
%        str2double(get(hObject,'String')) returns contents of edit_markersize as a double
test = str2double(get(hObject,'String'));
if isreal(test) && isfinite(test) && test > 0
    handles.markersize = test;
else
    errordlg('Invalid input','Input Error');
    set(handles.edit_markersize,'String',num2str(handles.markersize));
end
guidata(hObject, handles);
% --- Executes during object creation, after setting all properties.
function edit_markersize_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_markersize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pt1_x_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pt1_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pt1_x as text
%        str2double(get(hObject,'String')) returns contents of edit_pt1_x as a double
test = str2double(get(hObject,'String'));
if isreal(test) && isfinite(test)
    handles.pt1_x = test;
else
    errordlg('Invalid input','Input Error');
    set(handles.edit_pt1_x,'String',num2str(handles.pt1_x));
end
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_pt1_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pt1_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pt1_y_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pt1_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pt1_y as text
%        str2double(get(hObject,'String')) returns contents of edit_pt1_y as a double
test = str2double(get(hObject,'String'));
if isreal(test) && isfinite(test)
    handles.pt1_y = test;
else
    errordlg('Invalid input','Input Error');
    set(handles.edit_pt1_y,'String',num2str(handles.pt1_y));
end
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_pt1_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pt1_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pt2_x_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pt2_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pt2_x as text
%        str2double(get(hObject,'String')) returns contents of edit_pt2_x as a double
test = str2double(get(hObject,'String'));
if isreal(test) && isfinite(test)
    handles.pt2_x = test;
else
    errordlg('Invalid input','Input Error');
    set(handles.edit_pt2_x,'String',num2str(handles.pt2_x));
end
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_pt2_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pt2_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pt2_y_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pt2_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pt2_y as text
%        str2double(get(hObject,'String')) returns contents of edit_pt2_y as a double
test = str2double(get(hObject,'String'));
if isreal(test) && isfinite(test)
    handles.pt2_y = test;
else
    errordlg('Invalid input','Input Error');
    set(handles.edit_pt2_y,'String',num2str(handles.pt2_y));
end
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_pt2_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pt2_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pt2_id_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pt2_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pt2_id as text
%        str2double(get(hObject,'String')) returns contents of edit_pt2_id as a double

test = str2double(get(hObject,'String'));
if isfinite(test)
    test = uint16(test);
    if test >= 0 && test < 1023
        handles.pt2_id = test;
    else
        errordlg('Invalid input','Input Error');  
    end
else
    errordlg('Invalid input','Input Error');
end
set(handles.edit_pt2_id,'String',num2str(handles.pt2_id));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_pt2_id_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pt2_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_pt1_id_Callback(hObject, eventdata, handles)
% hObject    handle to edit_pt1_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_pt1_id as text
%        str2double(get(hObject,'String')) returns contents of edit_pt1_id as a double
test = str2double(get(hObject,'String'));
if isfinite(test)
    test = uint16(test);
    if test >= 0 && test < 1023
        handles.pt1_id = test;
    else
        errordlg('Invalid input','Input Error');  
    end
else
    errordlg('Invalid input','Input Error');
end
set(handles.edit_pt1_id,'String',num2str(handles.pt1_id));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_pt1_id_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_pt1_id (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_loopcount_Callback(hObject, eventdata, handles)
% hObject    handle to edit_loopcount (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_loopcount as text
%        str2double(get(hObject,'String')) returns contents of edit_loopcount as a double
test = str2double(get(hObject,'String'));
if isfinite(test)
    test = uint16(test);
    if test > 0 && test < 100
        handles.loopcount = test;
    else
        errordlg('Invalid input','Input Error');  
    end
else
    errordlg('Invalid input','Input Error');
end
set(handles.edit_loopcount,'String',num2str(handles.loopcount));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_loopcount_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_loopcount (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
