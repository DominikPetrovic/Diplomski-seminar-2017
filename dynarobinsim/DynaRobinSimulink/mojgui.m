function varargout = mojgui(varargin)
% MOJGUI MATLAB code for mojgui.fig
%      MOJGUI, by itself, creates a new MOJGUI or raises the existing
%      singleton*.
%
%      H = MOJGUI returns the handle to a new MOJGUI or the handle to
%      the existing singleton*.
%
%      MOJGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MOJGUI.M with the given input arguments.
%
%      MOJGUI('Property','Value',...) creates a new MOJGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mojgui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mojgui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mojgui

% Last Modified by GUIDE v2.5 07-May-2017 12:23:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mojgui_OpeningFcn, ...
                   'gui_OutputFcn',  @mojgui_OutputFcn, ...
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


% --- Executes just before mojgui is made visible.
function mojgui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mojgui (see VARARGIN)

% Choose default command line output for mojgui
% Update handles structure
guidata(hObject, handles);

global Plot1;
Plot1=handles.axes1;

global Plot2;
Plot2=handles.axes2;

otvaranje=1
assignin('base','handles',handles);

% UIWAIT makes mojgui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = mojgui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
izracuntocaka(0,0,0);



% --- Executes on slider movement.
function sliderduljine_Callback(hObject, eventdata, handles)
% hObject    handle to sliderduljine (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
%skretanje=get(handles.skretanje,'Value')

% if(get(handles.skretanje,'Value')==1)
%     izracuntocaka(0,1,0);
%     plotanjeskretanja(handles);
%     
%    pause(evalin('base','trajanje'));
%      izracuntocaka2(0,1,0);
%     plotanjeskretanja(handles);
%     
% end

duljina=get(hObject,'Value');
skretanje=0;

smjer=evalin('base','smjer');

% if(get(handles.naprijed,'Value')==1)
%     smjer=1
% end
% if(get(handles.rikverc,'Value')==1)
%     smjer=-1
% end

if(skretanje==1)
   izracuntocaka4(duljina,get(handles.sliderskretanja,'Value'),smjer)
else
izracuntocaka(duljina,skretanje,smjer);
end

if(skretanje==0)
    plotanjeravno(handles);
end
set(findobj('Tag','unos'),'String',num2str(duljina));
pomocnocrtanje();





% --- Executes during object creation, after setting all properties.
function sliderduljine_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderduljine (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end




% --- Executes on slider movement.
function brzina_Callback(hObject, eventdata, handles)
% hObject    handle to brzina (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

    trajanje=3.5-(get(hObject,'Value'));
  
    assignin('base', 'trajanje', trajanje);
    vectort=[0 linspace(0.25*trajanje,0.75*trajanje,5) trajanje];
    assignin('base', 'vectort', vectort);
    
set(findobj('Tag','trajanjekoraka'),'String',num2str(trajanje));




% --- Executes during object creation, after setting all properties.
function brzina_CreateFcn(hObject, eventdata, handles)
% hObject    handle to brzina (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end




function unos_Callback(hObject, eventdata, handles)
% hObject    handle to unos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of unos as text
%        str2double(get(hObject,'String')) returns contents of unos as a double


duljina = str2num(get(handles.unos,'String'));
if(duljina>=0.02 && duljina<=0.1)
    slider = findobj('Tag','sliderduljine');
    set(slider,'Value',duljina);   

end




% --- Executes during object creation, after setting all properties.
function unos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to unos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in rikverc.
function rikverc_Callback(hObject, eventdata, handles)
% hObject    handle to rikverc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rikverc
smjer=-1
assignin('base','smjer',smjer);

% --- Executes on button press in naprijed.
function naprijed_Callback(hObject, eventdata, handles)
% hObject    handle to naprijed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of naprijed
smjer=1
assignin('base','smjer',smjer);



% --- Executes on slider movement.
function sliderskretanja_Callback(hObject, eventdata, handles)
% hObject    handle to sliderskretanja (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
skretanje=get(hObject,'Value');
set(handles.koefskretanja,'String',skretanje);




% --- Executes during object creation, after setting all properties.
function sliderskretanja_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderskretanja (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



% --- Executes during object creation, after setting all properties.
function text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in trot.
function trot_Callback(hObject, eventdata, handles)
% hObject    handle to trot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
nacin=1
assignin('base','nacin',nacin);
assignin('base','skretanje',0);

% Hint: get(hObject,'Value') returns toggle state of trot


% --- Executes on button press in walk.
function walk_Callback(hObject, eventdata, handles)
% hObject    handle to walk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of walk
nacin=-1
assignin('base','nacin',nacin);
assignin('base','skretanje',0);




function trajanjekoraka_Callback(hObject, eventdata, handles)
% hObject    handle to trajanjekoraka (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of trajanjekoraka as text
%        str2double(get(hObject,'String')) returns contents of trajanjekoraka as a double
t=str2num(get(hObject,'String'));
if(t>=1 && t<=2)
set(handles.brzina,'Value',3-t);
end



% --- Executes during object creation, after setting all properties.
function trajanjekoraka_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trajanjekoraka (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function trenutak_Callback(hObject, eventdata, handles)
% hObject    handle to trenutak (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of trenutak as text
%        str2double(get(hObject,'String')) returns contents of trenutak as a double


% --- Executes during object creation, after setting all properties.
function trenutak_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trenutak (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in kreni.
function kreni_Callback(hObject, eventdata, handles)
% hObject    handle to kreni (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
duljina=get(handles.sliderduljine,'Value');
skretanje=get(handles.skretanje,'Value');
koefskretanja=get(handles.sliderskretanja,'Value');
simulacija=1;
assignin('base','simulacija',simulacija);


% naprijed=get(handles.naprijed,'Value');
% rikverc=get(handles.rikverc,'Value');
% if(naprijed==1)
%     smjer=1
% end
% if(rikverc==1)
%     smjer=-1
% end
smjer=evalin('base','smjer');
if(skretanje~=0)

assignin('base','skretanje',1);

% [izlaz1,izlaz2]=rezanje(get(handles.sliderskretanja,'Value'),get(handles.sliderduljine,'Value'),smjer);
% 
% for i=1:1:1
%     
%    
%     izracuntocaka3(izlaz1(1+(i-1)*28:i*28),handles);
%     runme
%     pause(24*evalin('base','trajanje'))
%     plotanjeskretanja(handles);
% end
% 
% 
% %nastavljanje ravno
% izracuntocaka(duljina,0,smjer);
% plotanjeskretanja(handles);
% runme

izracuntocaka4(duljina,koefskretanja,smjer);
plotanjeskretanja(handles);
key=1;
StvoriPutanju;
StvoriTrajektoriju;


else
izracuntocaka(duljina,0,smjer);
plotanjeskretanja(handles);
key=1;
StvoriPutanju;
StvoriTrajektoriju;
end

% if(simulacija==1)
%     LocalX=evalin('base','LocalX');
%     LocalY=evalin('base','LocalY');
%     LocalZ=evalin('base','LocalZ');
%     FLreal=evalin('base','FLreal');
%     FRreal=evalin('base','FRreal');
%     BLreal=evalin('base','BLreal');
%     BRreal=evalin('base','BRreal');
%     
%     kraj=1000*evalin('base','trajanje')
%     for i=1:1:kraj
%     if(LocalZ.signals.values(i,1)-FRreal.signals.values(i,3)~=0)
%         GRESKA=1
%         
%     end
%     end
% end

% --- Executes on button press in stani.
function stani_Callback(hObject, eventdata, handles)
% hObject    handle to stani (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
izracuntocaka(0,0,0);
%plotanjeskretanja(handles);
key=1;
StvoriPutanju;
StvoriTrajektoriju;
simulacija=0;
assignin('base','simulacija',simulacija);


% --- Executes on button press in okreni.
function okreni_Callback(hObject, eventdata, handles)
% hObject    handle to okreni (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
izlaz=plotanje;
okreni(izlaz)
key=1;
StvoriPutanju;
StvoriTrajektoriju;
% pause(20*evalin('base','trajanje'));
% izracuntocaka(0,0,0);
% runme;



function xcentra_Callback(hObject, eventdata, handles)
% hObject    handle to xcentra (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xcentra as text
%        str2double(get(hObject,'String')) returns contents of xcentra as a double
xc=str2num(get(hObject,'String'));
if(xc>=-0.1 && xc<=0.1)
assignin('base','xc',xc);
end


% --- Executes during object creation, after setting all properties.
function xcentra_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xcentra (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
xc=str2num(get(hObject,'String'));
if(xc>=-0.1 && xc<=0.1)
assignin('base','xc',xc);
end



function zcentra_Callback(hObject, eventdata, handles)
% hObject    handle to zcentra (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zcentra as text
%        str2double(get(hObject,'String')) returns contents of zcentra as a double
zc=str2num(get(hObject,'String'));
if(zc>=-0.23 && zc<=-0.13)
assignin('base','zc',zc);
end



% --- Executes during object creation, after setting all properties.
function zcentra_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zcentra (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
zc=str2num(get(hObject,'String'));
if(zc>=-0.23 && zc<=-0.13)
assignin('base','zc',zc);
end


% --- Executes on button press in probe.
function probe_Callback(hObject, eventdata, handles)
% hObject    handle to probe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
elipse(handles);
%plotanjeskretanja(handles);
%runme;



function koefskretanja_Callback(hObject, eventdata, handles)
% hObject    handle to koefskretanja (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of koefskretanja as text
%        str2double(get(hObject,'String')) returns contents of koefskretanja as a double


% --- Executes during object creation, after setting all properties.
function koefskretanja_CreateFcn(hObject, eventdata, handles)
% hObject    handle to koefskretanja (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xcentraz_Callback(hObject, eventdata, handles)
% hObject    handle to xcentraz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xcentraz as text
%        str2double(get(hObject,'String')) returns contents of xcentraz as a double
xcz=str2num(get(hObject,'String'));
if(xcz>=-0.1 && xcz<=0.1)
assignin('base','xcz',xcz);
end



% --- Executes during object creation, after setting all properties.
function xcentraz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xcentraz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
xcz=str2num(get(hObject,'String'));
if(xcz>=-0.1 && xcz<=0.1)
assignin('base','xcz',xcz);
end



function zcentraz_Callback(hObject, eventdata, handles)
% hObject    handle to zcentraz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zcentraz as text
%        str2double(get(hObject,'String')) returns contents of zcentraz as a double
zcz=str2num(get(hObject,'String'));
if(zcz>=-0.23 && zcz<=-0.13)
assignin('base','zcz',zcz);
end


% --- Executes during object creation, after setting all properties.
function zcentraz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zcentraz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
zcz=str2num(get(hObject,'String'));
if(zcz>=-0.23 && zcz<=-0.13)
assignin('base','zcz',zcz);
end


% --- Executes on slider movement.
function sliderz_Callback(hObject, eventdata, handles)
% hObject    handle to sliderz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
z=get(hObject,'Value');
assignin('base','zc',z);
assignin('base','zcz',z);
set(handles.zcentra,'String',num2str(z));
set(handles.zcentraz,'String',num2str(z));


% --- Executes during object creation, after setting all properties.
function sliderz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in proba.
function proba_Callback(hObject, eventdata, handles)
% hObject    handle to proba (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
proba();


% --- Executes during object creation, after setting all properties.
function slidervremena_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slidervremena (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slidervremena_Callback(hObject, eventdata, handles)
% hObject    handle to text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
trazenjetocke(get(hObject,'Value'));
plotanjevremena(handles);

set(findobj('Tag','trenutak'),'String',num2str(get(hObject,'Value')*evalin('base','trajanje')));
