%--------------------------------------------------------------------
%A simple MATLAB GUI for paparazzi autopilot log-file plotting
%Paparazzi Project [http://www.nongnu.org/paparazzi/]
%by Roman Krashhanitsa 28/10/2005
%adjustable parabeters:
% maxnum - increase if dialog window hangs up or doesnt refresh
% Nres - number or interpolated points in the plot, decrease to improve
% plotting performance or increase to improve resolution
%--------------------------------------------------------------------
function varargout = dialog(varargin)
% DIALOG M-file for dialog.fig
%      DIALOG, by itself, creates a new DIALOG or raises the existing
%      singleton*.
%
%      H = DIALOG returns the handle to a new DIALOG or the handle to
%      the existing singleton*.
%
%      DIALOG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DIALOG.M with the given input arguments.
%
%      DIALOG('Property','Value',...) creates a new DIALOG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before dialog_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to dialog_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help dialog

% Last Modified by GUIDE v2.5 10-Nov-2005 18:47:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @dialog_OpeningFcn, ...
                   'gui_OutputFcn',  @dialog_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin & isstr(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before dialog is made visible.
function dialog_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to dialog (see VARARGIN)

% Choose default command line output for dialog
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

if strcmp(get(hObject,'Visible'),'off')
    plot([0 1],[0 0]);
end

set(handles.ListSections,'Enable','off');
set(handles.ListFields,'Enable','off');
set(handles.ListDevices,'Enable','off');


% UIWAIT makes dialog wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = dialog_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
if ~isempty(handles)
    varargout{1} = handles.output;
end;

% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'comet(cos(1:.01:10))', 'bar(1:10)', 'plot(membrane)', 'surf(peaks)'});

% --- Executes on selection change in popupmenu3.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3



% --- Executes on button press in loadButton.
function loadButton_Callback(hObject, eventdata, handles)
% hObject    handle to loadButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global logData;
global labelsSections;

res=0;
[FileName,PathName,res] = uigetfile('*.log','Open Log file...');
if res~=0,
    %read protocol specification
    global nodeList;
    global labelsSections;
    global sectionsIndex;
    global labelsFields;
    global fieldsIndex;
    global id_Devices;
    try
        node=xmlread(fullfile(PathName,FileName));
    catch
        warning('File ',fullfile(PathName,FileName),' does not exist');
        close(gcf);
        return;
    end;
    
    %find name of the data file
    dataFileName=char(node.getFirstChild.getAttributes.item(0).getValue);
    
    %---read protocol---------------------------------
    set(handles.text1,'String','Reading protocol specification');
    notfound=1; %found messages or not
    finish=0; %structure traversing finished
    nodeList=node.getChildNodes;
    i=1; %level in the structure
    J=[0];
    val=''; 
    while notfound && ~finish,
        while J(i)<nodeList.getLength && notfound,
            drawnow; %flash event queue
            val=node.getNodeName;
            notfound=~strcmp(val,'message');
            if notfound
                while node.getChildNodes.getLength~=0 && notfound %go to the next level
                    nodeList=node.getChildNodes;
                    i=i+1;
                    J(i)=0;
                    node=nodeList.item(J(i));
                    val=node.getNodeName;
                    notfound=~strcmp(val,'message');
                end;
                J(i)=J(i)+1;
                if J(i)<nodeList.getLength 
                    node=nodeList.item(J(i));
                end;
            end;
        end;
        if i>=3 && notfound % if not a root node
            nodeList=node.getParentNode.getParentNode.getChildNodes;
            i=i-1;        
            J(i)=J(i)+1;
            if J(i)<nodeList.getLength 
                node=nodeList.item(J(i));
            end;
        else
            %nothing, will get caught by while loop
            finish=1;
        end;
    end;
    %make labels for the first list menu
    count=nodeList.getLength;
    labelsSections=[];
    lineSections=[];
    sectionsIndex=[];
    for j=0:count-1,
        if (nodeList.item(j).getNodeName=='message')
            lineSections=[lineSections,char(nodeList.item(j).getAttributes.item(1).getValue),'|'];
            sectionsIndex=[sectionsIndex,j];
            labelsSections=[labelsSections,{char(nodeList.item(j).getAttributes.item(1).getValue)}];
        end;
    end;
    lineSections=lineSections(1:max(size(lineSections))-1); %cut off last '|' character
    set(handles.ListSections,'String',lineSections);
    %make labels for the fields list (submenu)
    nn=get(handles.ListSections,'Value');
    nn=nn(1);
    childList=nodeList.item(sectionsIndex(nn)).getChildNodes;
    count=childList.getLength;
    labelsFields=[];
    for j=0:count-1,
        if childList.item(j).getNodeName=='field',
            attr=childList.item(j).getAttributes;
            cattr=attr.getLength; k=0;
            while k<cattr && ~strcmp(attr.item(k).getName,'name'),
                k=k+1;
            end;
            labelsFields=[labelsFields,char(attr.item(k).getValue),'|'];
            fieldsIndex=[fieldsIndex,k];
        end;
    end;
    labelsFields=labelsFields(1:max(size(labelsFields))-1); %cut off last '|' character
    set(handles.ListFields,'String',labelsFields);

    %---  read data  ----------------------------
    try
        fid=fopen(fullfile(PathName,dataFileName));
    catch
        warning('File ',fullfile(PathName,dataFileName),' does not exist');
    end;
    
    fseek(fid,0,'eof'); %ff to end
    endpos=ftell(fid); %find file size
    fseek(fid,0,'bof'); %rewind to start
    tline = fgetl(fid);
%     while ~(feof(fid) || ~isempty(findstr(tline,'<data>'))),
%         tline = fgetl(fid);
%     end;
    %read data
    logData=[]; 
    num=0; maxnum=100; %used to flush event queue
    while ~feof(fid),
        tline = fgetl(fid);
        [tok,tline]=strtok(tline);
        t=sscanf(tok,'%g'); %extract time
        [tok,tline]=strtok(tline);        
        plane=sscanf(tok,'%g'); %airplane id?
        [lab,tline]=strtok(tline); %extract message label
        fld=sscanf(tline,'%g'); %extract a vector of fields
        found=0; j=1; count=max(size(labelsSections));
        %find index of the message label in the protocol
        while j<count && ~found,
            found=~isempty(strmatch(lab,cell2mat(labelsSections(j))));
            j=j+1;
        end;
        j=j-1;
        if ~found,
            warning('Protocol specification in messages.xml does not match protocol in log file.');
        else % if found
            s=struct('time',t,'type',j,'plane_id',plane,'fields',fld);
            logData=[logData,s];
        end;
        pos=ftell(fid);
        set(handles.text1,'String',[num2str(double(pos)/double(endpos)*100.0,'%5.2f'),'%']);
        num=num+1;
        if num>maxnum,
            num=0;
            drawnow; %flash event queue
        end;
    end;
    
    %make labels for Device ID listbox
    id_Devices=[];
    count=max(size(logData));
    for j=1:count,
        k=1; nn=max(size(id_Devices));
        tag=logData(j).plane_id; notfound=1;
        while k<=nn && notfound,
            notfound=~(tag==id_Devices(k));
            if notfound, 
                k=k+1; 
            end;
        end;
        if notfound,
            id_Devices=[id_Devices,tag];
        end;
    end;
    labelsDevices=num2str(id_Devices(1));
    for j=2:max(size(id_Devices)),
        labelsDevices=[labelsDevices,'|',num2str(id_Devices(j))];
    end;
    set(handles.ListDevices,'String',labelsDevices);

    
    
    set(handles.text1,'String',FileName);
    set(handles.ListSections,'Enable','on');
    set(handles.ListFields,'Enable','on');
    set(handles.ListDevices,'Enable','on');
else
    %file was not selected
end;





% --- Executes during object creation, after setting all properties.
function ListFields_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ListFields (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on selection change in ListFields.
function ListFields_Callback(hObject, eventdata, handles)
% hObject    handle to ListFields (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns ListFields contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ListFields

%make labels for the fields list (submenu)






% --- Executes on button press in keepToggleButton.
function keepToggleButton_Callback(hObject, eventdata, handles)
% hObject    handle to keepToggleButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of keepToggleButton


% --- Executes during object creation, after setting all properties.
function ListSections_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ListSections (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end

% --- Executes on selection change in ListSections.
function ListSections_Callback(hObject, eventdata, handles)
% hObject    handle to ListSections (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns ListSections contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ListSections
global nodeList;
global labelsSections;
global sectionsIndex;
global labelsFields;
global fieldsIndex;

nn=get(handles.ListSections,'Value');
nn=nn(1);
childList=nodeList.item(sectionsIndex(nn)).getChildNodes;
count=childList.getLength;
labelsFields=[];
for j=0:count-1,
    if childList.item(j).getNodeName=='field',
        attr=childList.item(j).getAttributes;
        cattr=attr.getLength; k=0;
        while k<cattr && ~strcmp(attr.item(k).getName,'name'),
            k=k+1;
        end;
        labelsFields=[labelsFields,char(attr.item(k).getValue),'|'];
        fieldsIndex=[fieldsIndex,k];
    end;
end;
labelsFields=labelsFields(1:max(size(labelsFields))-1); %cut off last '|' character
set(handles.ListFields,'String',labelsFields);
set(handles.ListFields,'Value',1);

% --- Executes on button press in plotButton.
function plotButton_Callback(hObject, eventdata, handles)
% hObject    handle to plotButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global logData;
global id_Devices;
global x;
global y;

Nres=1000; % minimum number of points in the plot (resolution)
%if actual number of points if greater we dont need to change anything, but
%if it is less, interpolate using nearest neighbor as closest model of
%signals incoming to ground station. Previous value is used in the ap
%until a new value is obtained

axes(handles.axes1);

if ~get(handles.keepToggleButton,'Value')
    cla;
else
    hold on;
end;

n=get(handles.ListSections,'Value'); n=n(1);
m=get(handles.ListFields,'Value'); m=m(1);
g=get(handles.ListDevices,'Value'); dev=id_Devices(g(1));
x=[]; y=[];
len=max(size(logData));
for j=1:len,
    if logData(j).type==n && logData(j).plane_id==dev,
        x=[x;logData(j).time];
        y=[y;logData(j).fields(m)];
    end;
end;
 
if ~isempty(x) && ~isempty(y),
    MIN=min(x-x(1));
    MAX=max(x-x(1));
    X=MIN:(MAX-MIN)/Nres:MAX;
    if max(size(X))<=max(size(x)) %plot as it is
        plot(x-x(1),y);
    else
        plot(X,interp1(x-x(1),y,X,'nearest')); %interpolate using nearest neighbor
    end;
end;
%plot(1:0.01:25,rand*sin(1:0.01:25));
%axis([1 25 -1 1]);


% --- Executes on button press in printButton.
function printButton_Callback(hObject, eventdata, handles)
% hObject    handle to printButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printpreview(gcf);


% --- Executes on button press in edit.
function edit_Callback(hObject, eventdata, handles)
% hObject    handle to edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~get(handles.edit,'Value')
    plotedit off;
else
    plotedit(gcf);
end;


% --- Executes during object creation, after setting all properties.
function ListDevices_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ListDevices (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end


% --- Executes on selection change in ListDevices.
function ListDevices_Callback(hObject, eventdata, handles)
% hObject    handle to ListDevices (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns ListDevices contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ListDevices


