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

% Last Modified by GUIDE v2.5 18-Sep-2006 11:05:50

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

function [m,n]=set2Plot(handles,section,field)

axes(handles.axes1);

global labelsSections;
global sectionsIndex;
global labelsFields;
global fieldsIndex;
N=max(size(labelsSections));
n=1; while n<=N && ~strcmpi(labelsSections(n),section), n=n+1; end;
if strcmpi(labelsSections(n),section),
    set(handles.ListSections,'Value',n);
    ListSections_Callback(0, 0, handles);
    M=max(size(labelsFields));
    m=1; while m<=M && ~strcmpi(labelsFields(m),field), m=m+1; end;
    if strcmpi(labelsFields(m),field),
        return;
    end;
end;
m=0;
n=0;
return;

function [x,y]=setXY2plot(m,n,k)
%fetch xy data for section n, field m
global X0;
global logData;
global x;
global y;
x=[]; y=[];
len=max(size(logData));
last_time=0;
for j=1:len,
    if logData(j).type==n && logData(j).plane_id==k,
        if logData(j).time>last_time,
            x=[x;logData(j).time];
            y=[y;logData(j).fields(m)];
            last_time=x(max(size(x)));
        end;
    end;
end;

x=x-X0; % shift timer to start at the boot time

function h=plotlog(x,y)
%plot data for section n, field m, plane_id k

% minimum number of points in the plot (resolution)
%if actual number of points if greater we dont need to change anything, but
%if it is less, interpolate using nearest neighbor as closest model of
%signals incoming to ground station. Previous value is used in the ap
%until a new value is obtained
Nres=1000; 
h=0;
if ~isempty(x) && ~isempty(y),
    MIN=min(x);
    MAX=max(x);
    X=MIN:(MAX-MIN)/Nres:MAX;
    if max(size(X))<=max(size(x)) %plot as it is
        h=plot(x,y);
    else
        h=plot(x,y,'x');
        %plot(X,interp1(x,y,X,'nearest')); %interpolate using nearest neighbor
    end;
end;

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

global X0;
X0=0;

pp_home=getenv('PAPARAZZI_HOME');
if (max(size(pp_home))==0)
    warning('PAPARAZZI_HOME environment variable was not found. Using current dir..');
    pp_home=pwd; % otherwise use curent directory
else
    pp_home=fullfile(pp_home,'conf');
end;
%read protocol specification
global nodeList;
global labelsSections;
global sectionsIndex;
global labelsFields;
global fieldsIndex;
global X0;
X0=0;
try
    node=xmlread(fullfile(pp_home,'messages.xml'));
catch
    warning('messages.xml not found. trying conf/messages.xml...')
    try
        node=xmlread(fullfile(pp_home,'conf/messages.xml'));
    catch
        warning('messages.xml not found. Exiting...');
        close(gcf);
%        delete(handles.figure1);
        return;
    end;
end;
nodeList=node.getChildNodes.item(0).getChildNodes;
%make labels for the first list menu
count=nodeList.getLength;
labelsSections=[];
lineSections=[];
sectionsIndex=[];
j=0; found=false;
while j<count && ~found,
    if (nodeList.item(j).getNodeType == nodeList.item(j).ELEMENT_NODE & ...
        strcmp(nodeList.item(j).getNodeName,'class') & ...
        strcmp(nodeList.item(j).getAttributes.getNamedItem('name').getValue,'telemetry'))
        found=true;
    else j=j+1;
    end;
end;
nodeList=nodeList.item(j).getChildNodes;
count=nodeList.getLength;
for j=0:count-1,
    if (nodeList.item(j).getNodeType == nodeList.item(j).ELEMENT_NODE & ...
        strcmp(nodeList.item(j).getNodeName,'message'))
        lineSections=[lineSections,char(nodeList.item(j).getAttributes.getNamedItem('name').getValue),'|'];
        sectionsIndex=[sectionsIndex,j];
        labelsSections=[labelsSections,{char(nodeList.item(j).getAttributes.getNamedItem('name').getValue)}];
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
    if strcmp(childList.item(j).getNodeName,'field'),
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
%     try
%         node=xmlread(fullfile(PathName,FileName));
%     catch
%         warning('File ',fullfile(PathName,FileName),' does not exist');
%         close(gcf);
%         return;
%     end;
%     
%     %find name of the data file
%     dataFileName=char(node.getFirstChild.getAttributes.item(0).getValue);
%     %find ground altitude
%     global ground_alt;
%     j=0; 
%     found=0
%     while ~found && j<=node.getFirstChild.getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(1).getAttributes.getLength,
%         if ~strcmpi(node.getFirstChild.getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(1).getAttributes.item(j).getName,'GROUND_ALT'),
%             j=j+1;
%             found=1;
%         end;
%     end;
%     if j<=node.getFirstChild.getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(1).getAttributes.getLength,
%         ground_alt=str2num(char(node.getFirstChild.getChildNodes.item(1).getChildNodes.item(1).getChildNodes.item(1).getAttributes.item(j).getValue));
%     else ground_alt=0;
%     end;
%     
%     
%     %---read protocol---------------------------------
%     set(handles.text1,'String','Reading protocol specification');
%     notfound=1; %found messages or not
%     finish=0; %structure traversing finished
%     nodeList=node.getChildNodes;
%     i=1; %level in the structure
%     J=[0];
%     val=''; 
%     while notfound && ~finish,
%         while J(i)<nodeList.getLength && notfound,
%             drawnow; %flash event queue
%             val=node.getNodeName;
%             notfound=~strcmp(val,'message');
%             if notfound
%                 while node.getChildNodes.getLength~=0 && notfound %go to the next level
%                     nodeList=node.getChildNodes;
%                     i=i+1;
%                     J(i)=0;
%                     node=nodeList.item(J(i));
%                     val=node.getNodeName;
%                     notfound=~strcmp(val,'message');
%                 end;
%                 J(i)=J(i)+1;
%                 if J(i)<nodeList.getLength 
%                     node=nodeList.item(J(i));
%                 end;
%             end;
%         end;
%         if i>=3 && notfound % if not a root node
%             nodeList=node.getParentNode.getParentNode.getChildNodes;
%             i=i-1;        
%             J(i)=J(i)+1;
%             if J(i)<nodeList.getLength 
%                 node=nodeList.item(J(i));
%             end;
%         else
%             %nothing, will get caught by while loop
%             finish=1;
%         end;
%     end;
%     %make labels for the first list menu
%     count=nodeList.getLength;
%     labelsSections=[];
%     lineSections=[];
%     sectionsIndex=[];
%     for j=0:count-1,
%         if (nodeList.item(j).getNodeName=='message')
%             lineSections=[lineSections,char(nodeList.item(j).getAttributes.item(1).getValue),'|'];
%             sectionsIndex=[sectionsIndex,j];
%             labelsSections=[labelsSections,{char(nodeList.item(j).getAttributes.item(1).getValue)}];
%         end;
%     end;
%     lineSections=lineSections(1:max(size(lineSections))-1); %cut off last '|' character
%     set(handles.ListSections,'String',lineSections);
%     %make labels for the fields list (submenu)
%     nn=get(handles.ListSections,'Value');
%     nn=nn(1);
%     childList=nodeList.item(sectionsIndex(nn)).getChildNodes;
%     count=childList.getLength;
%     labelsFields=[];
%     for j=0:count-1,
%         if childList.item(j).getNodeName=='field',
%             attr=childList.item(j).getAttributes;
%             cattr=attr.getLength; k=0;
%             while k<cattr && ~strcmp(attr.item(k).getName,'name'),
%                 k=k+1;
%             end;
%             labelsFields=[labelsFields,char(attr.item(k).getValue),'|'];
%             fieldsIndex=[fieldsIndex,k];
%         end;
%     end;
%     labelsFields=labelsFields(1:max(size(labelsFields))-1); %cut off last '|' character
%     set(handles.ListFields,'String',labelsFields);

if res~=0,
    try
        fid=fopen(fullfile(PathName,FileName));
    catch
        error(lasterror);
    end;
    %find beginning of data
    fseek(fid,0,'eof'); %ff to end
    endpos=ftell(fid); %find file size
    fseek(fid,0,'bof'); %rewind to start
    tline = fgetl(fid);
    while ~(feof(fid) || ~isempty(findstr(tline,'data_file='))),
        tline = fgetl(fid);
    end;

    [name,value]=strread(tline,'%q%q','delimiter','"');
    dataFileName=value{2};
    fclose(fid);
    %---  read data  ----------------------------
    try
        fid=fopen(fullfile(PathName,dataFileName));
    catch
        warning('File ',fullfile(PathName,dataFileName),' does not exist');
    end;
    
    fseek(fid,0,'eof'); %ff to end
    endpos=ftell(fid); %find file size
    fseek(fid,0,'bof'); %rewind to start
    %tline = fgetl(fid);
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
        while j<count+1 && ~found,
            %found=~isempty(strmatch(lab,cell2mat(labelsSections(j))));
            found=strcmp(lab,cell2mat(labelsSections(j)));            
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
    global id_Devices;
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

j=1; 
while (~strcmp(labelsSections{j},'BOOT') && j<max(size(labelsSections)) )
    j=j+1;
end;
k=1;
while logData(k).type~=j && k<max(size(logData))
    k=k+1;
end;
if (k<max(size(logData)))
    X0=logData(k).time;
else
    warning('BOOT message not found.. Corrupted log?');
end;
end; %function
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
% global nodeList;
% global labelsSections;
% global sectionsIndex;
% global labelsFields;
% global fieldsIndex;
% 
% nn=get(handles.ListSections,'Value');
% nn=nn(1);
% childList=nodeList.item(sectionsIndex(nn)).getChildNodes;
% count=childList.getLength;
% labelsFields=[];
% for j=0:count-1,
%     if childList.item(j).getNodeName=='field',
%         attr=childList.item(j).getAttributes;
%         cattr=attr.getLength; k=0;
%         while k<cattr && ~strcmp(attr.item(k).getName,'name'),
%             k=k+1;
%         end;
%         labelsFields=[labelsFields,char(attr.item(k).getValue),'|'];
%         fieldsIndex=[fieldsIndex,k];
%     end;
% end;
% labelsFields=labelsFields(1:max(size(labelsFields))-1); %cut off last '|' character
% set(handles.ListFields,'String',labelsFields);
% set(handles.ListFields,'Value',1);



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
lineFields=[];
fieldsIndex=[];
for j=0:count-1,
    if childList.item(j).getNodeName=='field',
        attr=childList.item(j).getAttributes;
        cattr=attr.getLength; k=0;
        while k<cattr && ~strcmp(attr.item(k).getName,'name'),
            k=k+1;
        end;
        labelsFields=[labelsFields,{char(attr.item(k).getValue)}];
        lineFields=[lineFields,char(attr.item(k).getValue),'|'];
        fieldsIndex=[fieldsIndex,k];
    end;
end;
lineFields=lineFields(1:max(size(lineFields))-1); %cut off last '|' character
set(handles.ListFields,'String',lineFields);
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
n=get(handles.ListSections,'Value'); 
n=n(1);
m=get(handles.ListFields,'Value'); m=m(1);
k=get(handles.ListDevices,'Value');
global id_Devices;
k=id_Devices(k);

[x,y]=setXY2plot(m,n,k);
h=plotlog(x,y);

t1=str2num(get(handles.edit1,'String'));
t2=str2num(get(handles.edit2,'String'));
if t1~=t2 && t1<t2,
    xlim([t1 t2]);
    axis 'auto y';
end;

fig=figure('Position',[250, 280, 600,240]); 

ax=gca;
[x,y]=setXY2plot(m,n,k);
axes(ax);

t1=str2num(get(handles.edit1,'String'));
t2=str2num(get(handles.edit2,'String'));
if t1~=t2 && t1<t2,
    [c,j1]=min(abs(x-t1)); j1
    [c,j2]=min(abs(x-t2)); j2
    p=plotlog(x(j1:j2)-x(j1),y(j1:j2));
    xlim([0 t2-t1]);
    axis 'auto y';
else
    p=plotlog(x,y);
    axis 'auto y';
end;

setTemplate(p,ax,fig);

xlabel('Time, s');
ylabel('Throttle');



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


% --- Executes on button press in zoomin.
function zoomin_Callback(hObject, eventdata, handles)
% hObject    handle to zoomin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value')==get(hObject,'Max')
    zoom on;
else
    zoom off;
end;

% --- Executes on button press in zoomout.
function zoomout_Callback(hObject, eventdata, handles)
% hObject    handle to zoomout (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value')==get(hObject,'Min')
    zoom on;
else
    zoom off;
end;


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes on button press in rotate3d.
function rotate3d_Callback(hObject, eventdata, handles)
% hObject    handle to rotate3d (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rotate3d

if get(hObject,'Value')==get(hObject,'Min')
    rotate3d off;
else
    rotate3d on;
end;



% --- Executes on button press in roll_button_button.
function roll_button_Callback(hObject, eventdata, handles)
% hObject    handle to roll_button_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x;
global y;
global id_Devices;

axes(handles.axes1);
[m,n]=set2Plot(handles,'attitude','phi');
k=get(handles.ListDevices,'Value'); k=id_Devices(k);
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    h=plotlog(x,y);
    set(h,'LineWidth',0.5);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
end;
hold on;
[m,n]=set2Plot(handles,'desired','roll');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    h=plotlog(x,y*180/pi);
    set(h,'LineWidth',2.0);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
end;
hold off;

t1=str2num(get(handles.edit1,'String'));
t2=str2num(get(handles.edit2,'String'));
if t1~=t2 && t1<t2,
    axis([t1 t2 -inf inf]); axis 'auto y';
end;

legend('estimated\_roll','desired\_roll');

figure('Position',[250, 280, 600,270]);
hh=gca;
[m,n]=set2Plot(handles,'desired','roll');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    axes(hh);
    h=plotlog(x,y*180/pi);
    set(h,'LineWidth',3.0);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
    set(h,'Color',[0.6 0.6 1]);    
end;
hold on;
[m,n]=set2Plot(handles,'attitude','phi');
k=get(handles.ListDevices,'Value'); k=id_Devices(k);
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    axes(hh);
    h=plotlog(x,y);
    set(h,'LineWidth',1.);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
    set(h,'Color',[0 0 0]);        
end;
hold off;

t1=str2num(get(handles.edit1,'String'));
t2=str2num(get(handles.edit2,'String'));
if t1~=t2 && t1<t2,
    axis([t1 t2 -inf inf]); axis 'auto y';
end;

legend('Desired roll','Estimated roll');
xlabel('Time, s');
ylabel('Angle, deg');
set(hh,'FontName','Times');
set(hh,'FontSize',9);
set(get(hh,'XLabel'),'FontName','Times','FontSize',9);
set(get(hh,'YLabel'),'FontName','Times','FontSize',9);

% --- Executes on button press in pitch_button.
function pitch_button_Callback(hObject, eventdata, handles)
% hObject    handle to pitch_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x;
global y;
global id_Devices;

axes(handles.axes1);

k=get(handles.ListDevices,'Value'); k=id_Devices(k);

[m,n]=set2Plot(handles,'attitude','theta');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    h=plotlog(x,y);
    set(h,'LineWidth',0.5);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
end;
hold on;
[m,n]=set2Plot(handles,'desired','pitch');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    h=plotlog(x,y);
    set(h,'LineWidth',2.0);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
end;
hold off;

t1=str2num(get(handles.edit1,'String'));
t2=str2num(get(handles.edit2,'String'));
if t1~=t2 && t1<t2,
    axis([t1 t2 -inf inf]); axis 'auto y';
end;

legend('estimated\_pitch','desired\_pitch');

% --- Executes on button press in heading_button.
function heading_button_Callback(hObject, eventdata, handles)
% hObject    handle to heading_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x;
global y;
global id_Devices;

axes(handles.axes1);
k=get(handles.ListDevices,'Value'); k=id_Devices(k);

[m,n]=set2Plot(handles,'GPS','course');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    h=plotlog(x,y*0.1);
    set(h,'LineWidth',0.5);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
end;
hold on;
[m,n]=set2Plot(handles,'navigation','desired_course');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    h=plotlog(x,y*0.1);
    set(h,'LineWidth',2.0);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
end;
hold off;

t1=str2num(get(handles.edit1,'String'));
t2=str2num(get(handles.edit2,'String'));
if t1~=t2 && t1<t2,
    axis([t1 t2 -inf inf]); axis 'auto y';
end;

legend('estimated\_course','desired\_course');


% --- Executes on button press in altitude_button.
function altitude_button_Callback(hObject, eventdata, handles)
% hObject    handle to altitude_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x;
global y;
global id_Devices;

axes(handles.axes1);
k=get(handles.ListDevices,'Value'); k=id_Devices(k);

[m,n]=set2Plot(handles,'desired','desired_altitude');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    h=plotlog(x,y);
    set(h,'LineWidth',3.0);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
end;
hold on;
[m,n]=set2Plot(handles,'GPS','alt');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    h=plotlog(x,y/100);
    set(h,'LineWidth',1);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
end;
hold off;

t1=str2num(get(handles.edit1,'String'));
t2=str2num(get(handles.edit2,'String'));
if t1~=t2 && t1<t2,
    axis([t1 t2 -inf inf]); axis 'auto y';
end;

legend('estimated\_altitude','desired\_altitude');

figure('Position',[250, 280, 600,270]);

hh=gca;

axes(hh);
k=get(handles.ListDevices,'Value'); k=id_Devices(k);

[m,n]=set2Plot(handles,'desired','desired_altitude');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    axes(hh);
    h=plotlog(x,y);
    set(h,'LineWidth',3.0);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
    set(h,'Color',[0.6,0.6,1]);
end;
hold on;
[m,n]=set2Plot(handles,'GPS','alt');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    axes(hh);
    h=plotlog(x,y/100);
    set(h,'LineWidth',1);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
    set(h,'Color',[0,0,0]);
end;
hold off;

t1=str2num(get(handles.edit1,'String'));
t2=str2num(get(handles.edit2,'String'));
if t1~=t2 && t1<t2,
    axis([t1 t2 -inf inf]); axis 'auto y';
end;

legend('Desired altitude','Estimated altitude');
xlabel('Time, s');
ylabel('Altitude, m');
set(hh,'FontName','Times','FontSize',9);
set(get(hh,'XLabel'),'FontName','Times','FontSize',9);
set(get(hh,'YLabel'),'FontName','Times','FontSize',9);

% --- Executes on button press in traj_button.
function traj_button_Callback(hObject, eventdata, handles)
% hObject    handle to traj_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x;
global y;
global z;
global t;
global id_Devices;

axes(handles.axes1);

xx=[];
yy=[];
zz=[];
k=get(handles.ListDevices,'Value'); k=id_Devices(k);
format long;
[m,n]=set2Plot(handles,'navigation','pos_x');
%[m,n]=set2Plot(handles,'GPS','utm_east');
if m*n~=0,
    [tx,xx]=setXY2plot(m,n,k);
end;
[m,n]=set2Plot(handles,'navigation','pos_y');
%[m,n]=set2Plot(handles,'GPS','utm_north');
if m*n~=0,
    [ty,yy]=setXY2plot(m,n,k);
%    yy=interp1(ty,yy,tx,'spline');
end;
[m,n]=set2Plot(handles,'GPS','alt');
if m*n~=0,
    [tz,zz]=setXY2plot(m,n,k);
    zz=interp1(tz,zz,tx,'nearest');
end;

x=xx;
y=yy;
global ground_alt;
z=zz/100-ground_alt;
t=tx;

if ~isempty(x) && ~isempty(y) && ~isempty(z) && ~isempty(t),
    t1=str2num(get(handles.edit1,'String'));
    t2=str2num(get(handles.edit2,'String'));
    if t1~=t2 && t1<t2 && t1<=t(max(size(t)-1)),
        N=max(size(t));
        n1=1; while n1<=N && t(n1)<t1, n1=n1+1; end;
        n2=n1+1; while n2<=N && t(n2)<t2, n2=n2+1; end;
        h=plot3(x(n1:n2),y(n1:n2),z(n1:n2));
    else
        h=plot3(x,y,z);
    end;
    set(h,'LineWidth',1.0);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    box;
    grid on;
end;

figure('Position',[250, 280, 600,270]);

hh=gca;
axes(hh);

if ~isempty(x) && ~isempty(y) && ~isempty(z) && ~isempty(t),
    t1=str2num(get(handles.edit1,'String'));
    t2=str2num(get(handles.edit2,'String'));
    if t1~=t2 && t1<t2 && t1<=t(max(size(t)-1)),
        N=max(size(t));
        n1=1; while n1<=N && t(n1)<t1, n1=n1+1; end;
        n2=n1+1; while n2<=N && t(n2)<t2, n2=n2+1; end;
        axes(hh);
        h=plot3(x(n1:n2),y(n1:n2),z(n1:n2));
    else
        axes(hh);
        h=plot3(x,y,z);
    end;
    set(h,'LineWidth',1.0);t
    set(h,'Marker','none');
    set(h,'LineStyle','-');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    box;
    grid on;
end;


% --- Executes on button press in vel_button.
function vel_button_Callback(hObject, eventdata, handles)
% hObject    handle to vel_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x;
global y;
global z;
global t;


axes(handles.axes1);
k=get(handles.ListDevices,'Value'); 
global id_Devices;
k=id_Devices(k);

[m,n]=set2Plot(handles,'GPS','speed');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    h=plotlog(x,y/100);
    set(h,'LineWidth',0.5);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
end;
hold on;

xx=[];
yy=[];
zz=[];

[m,n]=set2Plot(handles,'navigation','pos_x');
if m*n~=0,
    [tx,xx]=setXY2plot(m,n,k);
end;
[m,n]=set2Plot(handles,'navigation','pos_y');
if m*n~=0,
    [ty,yy]=setXY2plot(m,n,k);
    yy=interp1(ty,yy,tx);
end;
[m,n]=set2Plot(handles,'GPS','alt');
if m*n~=0,
    [tz,zz]=setXY2plot(m,n,k);
    zz=interp1(tz,zz,tx)/100;
end;
v=[];N=max(size(xx));
for j=2:N, v=[v sqrt((xx(j)-xx(j-1))^2+(yy(j)-yy(j-1))^2+(zz(j)-zz(j-1))^2)/(tx(j)-tx(j-1))]; end;
y=v;
x=tx(2:N);
h=plotlog(x,y);
if ~isempty(x) && ~isempty(y),
    t1=str2num(get(handles.edit1,'String'));
    t2=str2num(get(handles.edit2,'String'));
    if t1~=t2 && t1<t2 && t1<=t(max(size(t)-1)),
        axis([t1 t2 -inf inf]); axis 'auto y';
    end;
    set(h,'LineWidth',2.0);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
end;
hold off;
legend('GPS\_speed','computed\_speed');

% --- Executes on button press in gaz_button.
function gaz_button_Callback(hObject, eventdata, handles)
% hObject    handle to gaz_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x;
global y;

axes(handles.axes1);
k=get(handles.ListDevices,'Value');
global id_Devices;
k=id_Devices(k);


[m,n]=set2Plot(handles,'servos','thrust');
if m*n~=0, 
    [x,y]=setXY2plot(m,n,k);
    h=plotlog(x,y);
    set(h,'LineWidth',0.5);
    set(h,'Marker','none');
    set(h,'LineStyle','-');
end;
t1=str2num(get(handles.edit1,'String'));
t2=str2num(get(handles.edit2,'String'));
if t1~=t2 && t1<t2,
    axis([t1 t2 -inf inf]); axis 'auto y';
end;

% --- Executes on button press in ap_mode_pushbutton_pushbutton.
function ap_mode_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to ap_mode_pushbutton_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
xl=xlim;
yl=ylim;
xscale=xl(2)-xl(1);
yscale=yl(2)-yl(1);


k=get(handles.ListDevices,'Value');
global id_Devices;
k=id_Devices(k);


[m,n]=set2Plot(handles,'PPRZ_MODE','ap_mode');
if m*n==0, 
    return;
end;
n1=1;
[x,y]=setXY2plot(m,n,k);
N=max(size(x));
while x(n1)<xl(1) && n1<N, n1=n1+1; end;
while x(n1)<xl(2) && n1<N,
    m1=y(n1+1);
    C=[0 0 0]; %color
    switch m1
        case 0
            C=[1 0.3 0];
            txt='manual';
        case 1
            C=[1 0.5 0];
            txt='auto1';
        case 2
            C=[1 0.7 0];
            txt='auto2';
        case 3
            C=[1 1 0];
            txt='home';
    end;    
    n2=n1+1; while y(n2)==m1 && x(n2)<xl(2) && n2<N, n2=n2+1; end;
    hold on;
    if x(n1)<xl(1), t1=xl(1)+0.01*xscale; else t1=x(n1); end;
    if x(n2)>xl(2), t2=xl(2)-0.01*xscale; else t2=x(n2); end;
    line([t1 t2],[yl(1)+0.1*yscale yl(1)+0.1*yscale],'LineWidth',0.5,'Color',C);   
    if x(n1)>=xl(1), patch([1,0,1]*xscale*0.01+x(n1),[-1 0 1]*yscale*0.01+yl(1)+0.1*yscale,C); end;
    if x(n2)<=xl(2), patch(-1*[1,0,1]*xscale*0.01+x(n2),[-1 0 1]*yscale*0.01+yl(1)+0.1*yscale,C); end;
    if t2-t1>0.15*xscale, text(t1+0.5*(t2-t1),yl(1)+0.1*yscale,txt,...
            'EdgeColor',C,'BackgroundColor',get(handles.axes1,'Color'),'HorizontalAlignment','Center'); end;
    hold off;
    n1=n2;
end;
legend_handle=legend;
if ~isempty(legend_handle), legend(legend_handle); end; %refresh legend



% --- Executes during object creation, after setting all properties.
function vel_button_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vel_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called




function setTemplate(pl,ax,fig)
set(fig,'Position',[250, 280, 600,240])
set(pl,'LineWidth',1,'Color',[0,0,0]); 
set(pl,'LineWidth',1.0);
set(pl,'Marker','x');
set(pl,'LineStyle','-');
set(ax,'YLimMode','auto');
set(ax,'LineWidth',1.0);
set(ax,'LineStyle','-');
set(ax,'FontName','Times');
set(ax,'FontSize',10);
box on;
