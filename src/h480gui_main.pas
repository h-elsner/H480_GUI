        {********************************************************}
        {                                                        }
        {       Read and send data to Yuneec CGO3+ camera        }
        {                                                        }
        {       Copyright (c) 2025         Helmut Elsner         }
        {                                                        }
        {       Compiler: FPC 3.2.3   /    Lazarus 3.7           }
        {                                                        }
        { Pascal programmers tend to plan ahead, they think      }
        { before they type. We type a lot because of Pascal      }
        { verboseness, but usually our code is right from the    }
        { start. We end up typing less because we fix less bugs. }
        {           [Jorge Aldo G. de F. Junior]                 }
        {********************************************************}

(*
This source is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free
Software Foundation; either version 2 of the License, or (at your option)
any later version.

This code is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details.

A copy of the GNU General Public License is available on the World Wide Web
at <http://www.gnu.org/copyleft/gpl.html>. You can also obtain it by writing
to the Free Software Foundation, Inc., 51 Franklin Street - Fifth Floor,
Boston, MA 02110-1335, USA.


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*******************************************************************************)

{This unit needs following additional components:
- Synapse
- Industrial stuff

Also the units mav_def and mav_msg from repository "Common units" are needed:
https://github.com/h-elsner/common_units
}

unit H480GUI_main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, ExtCtrls, StdCtrls,
  lclintf, lcltype, Buttons, ActnList, Menus, Process, XMLPropStorage, ComCtrls,
  Grids, ValEdit, Spin, TAGraph, TATypes, TASeries, TAChartUtils, TAGeometry,
  TARadialSeries, TASources, TAIntervalSources, synaser,
  clipbrd, mav_def, mav_msg, Types, FileUtil;

type

  { TForm1 }

  TForm1 = class(TForm)
    acConnect: TAction;
    acClose: TAction;
    acDisconnect: TAction;
    acScanPorts: TAction;
    acSaveGUItext: TAction;
    acCopySerial: TAction;
    acEnableTesting: TAction;
    acDeleteText: TAction;
    acLoadLicense: TAction;
    ActionList1: TActionList;
    AddValueLineSeries1: TLineSeries;
    BarSatSNR: TBarSeries;
    btnHeightLimit: TBitBtn;
    btnGeoFence: TBitBtn;
    btnLicense: TBitBtn;
    btnDeleteText: TBitBtn;
    btnDisableNFZ: TButton;
    btnEnableNFZ: TButton;
    btnEnableTesting: TButton;
    btnSaveMsg: TBitBtn;
    btnDisconnect: TBitBtn;
    btnClose: TBitBtn;
    btnConnect: TBitBtn;
    btnFrontCali: TButton;
    btnTurnAll: TButton;
    cbHighRPM: TCheckBox;
    cbLimitMsg: TCheckBox;
    cbPort: TComboBox;
    cbRecord: TCheckBox;
    cbSensor: TCheckBox;
    cbSpeed: TComboBox;
    chAddValue: TChart;
    ChartSatSNR: TChart;
    DateTimeIntervalChartSource1: TDateTimeIntervalChartSource;
    gbAcc: TGroupBox;
    gbBaro: TGroupBox;
    gbDeviceInfo: TGroupBox;
    gbGeoFence: TGroupBox;
    gbGyro: TGroupBox;
    gbMag: TGroupBox;
    gbMessages: TGroupBox;
    gbMotors: TGroupBox;
    gbOrientation: TGroupBox;
    gbPosition: TGroupBox;
    gbSysStatus: TGroupBox;
    gbVelocity: TGroupBox;
    GUItext: TMemo;
    Image1: TImage;
    lbIGLONASSsats: TLabel;
    lbIGPSOK: TLabel;
    lbIGPSsats: TLabel;
    lbIIESCOK: TLabel;
    lbIIMUOK: TLabel;
    lbIRSOK: TLabel;
    lbISBASsats: TLabel;
    lbISonar: TLabel;
    lblCurrentValue: TLabel;
    lblEnableTesting: TLabel;
    lblFCtime: TLabel;
    lblFCtimeGPS: TLabel;
    lblGeoFence: TLabel;
    lblGeoFenceVal: TLabel;
    lblHeightLimit: TLabel;
    lblHeightLimitVal: TLabel;
    lblNewValue: TLabel;
    lblNotUsed: TLabel;
    lblOK: TLabel;
    lblOtherSats: TLabel;
    lblSatUsed: TLabel;
    lblSysTime: TLabel;
    mnLoadLicense: TMenuItem;
    mnEnableTesting: TMenuItem;
    mnCopySerial: TMenuItem;
    mnSaveGUItext: TMenuItem;
    OpenDialog: TOpenDialog;
    pcGUI: TPageControl;
    picGLONASS: TImage;
    picGPS: TImage;
    picMotors: TImage;
    picSBAS: TImage;
    rgAddValue: TRadioGroup;
    SatPolar: TChart;
    SatPolarSeries: TPolarSeries;
    SensorLEDPanel: TPanel;
    Separator1: TMenuItem;
    ImageList1: TImageList;
    mnLicense: TPopupMenu;
    SatPolarSource: TListChartSource;
    SatSNRBarSource: TListChartSource;
    SaveDialog1: TSaveDialog;
    Separator2: TMenuItem;
    shapeESCOK: TShape;
    shapeGPSOK: TShape;
    shapeIMUOK: TShape;
    shapeNotUsed: TShape;
    shapeRSOK: TShape;
    shapeSonar: TShape;
    shapeUsed: TShape;
    speGeoFence: TSpinEdit;
    speHeightLimit: TSpinEdit;
    StatusBar1: TStatusBar;
    timerSensors: TTimer;
    timerGUI: TTimer;
    tsGPSinfo: TTabSheet;
    tsSensorInfo: TTabSheet;
    tsSettings: TTabSheet;
    upperPanel: TPanel;
    vleAcc: TValueListEditor;
    vleBaro: TValueListEditor;
    vleGyro: TValueListEditor;
    vleMag: TValueListEditor;
    vleOrientation: TValueListEditor;
    vlePosition: TValueListEditor;
    vleSysStatus: TValueListEditor;
    vleSystem: TValueListEditor;
    vleVelocity: TValueListEditor;
    XMLPropStorage1: TXMLPropStorage;

    procedure acCloseExecute(Sender: TObject);
    procedure acConnectExecute(Sender: TObject);
    procedure acCopySerialExecute(Sender: TObject);
    procedure acDeleteTextExecute(Sender: TObject);
    procedure acDisconnectExecute(Sender: TObject);
    procedure acEnableTestingExecute(Sender: TObject);
    procedure acLoadLicenseExecute(Sender: TObject);
    procedure acSaveGUItextExecute(Sender: TObject);
    procedure acScanPortsExecute(Sender: TObject);
    procedure btnDisableNFZClick(Sender: TObject);
    procedure btnEnableNFZClick(Sender: TObject);
    procedure btnGeoFenceClick(Sender: TObject);
    procedure btnHeightLimitClick(Sender: TObject);
    procedure btnTurnAllClick(Sender: TObject);
    procedure cbPortDblClick(Sender: TObject);
    procedure FormActivate(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure GUItextMouseWheelDown(Sender: TObject; Shift: TShiftState;
      MousePos: TPoint; var Handled: Boolean);
    procedure GUItextMouseWheelUp(Sender: TObject; Shift: TShiftState;
      MousePos: TPoint; var Handled: Boolean);
    procedure Image1Click(Sender: TObject);
    procedure picMotorsMouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure rgAddValueClick(Sender: TObject);
    procedure SatPolarAfterDrawBackWall(ASender: TChart; ACanvas: TCanvas;
      const ARect: TRect);
    procedure timerGUITimer(Sender: TObject);
    procedure timerSensorsTimer(Sender: TObject);
    procedure vleAccPrepareCanvas(Sender: TObject; aCol, aRow: Integer;
      aState: TGridDrawState);
    procedure vleVelocityPrepareCanvas(Sender: TObject; aCol, aRow: Integer;
      aState: TGridDrawState);

  private
    procedure StopAllTimer;
    procedure ResetSensorsStatus;

    procedure GridPrepare(var grid: TStringGrid; const NumRows: byte);
    procedure WriteGUIvalueListHeader;
    procedure ClearGUI;
    procedure FillGUIPosition24(const sats: TGPSdata);
    procedure FillGPS_STATUS(const sats: TGPSdata);
    procedure FillGUIPosition33(const sats: TGPSdata);
    procedure FillGUIIMU(const data: THWstatusData);
    procedure FillGUI_SYS_STATUS(const msg: TMAVmessage; var data: TGPSdata);
    procedure FillSENSOR_OFFSETS(const data: THWstatusData);
    procedure FillAttitude(data: TAttitudeData);
    procedure FillEKF_STATUS_REPORT(data: TAttitudeData);

    procedure CreateSatSNRBarChart(const sats: TGPSdata);
    procedure CreateSatPolarDiagram(const sats: TGPSdata);
    procedure PrepareSatSNRBarChart(const LeftCaption: string);  {My settings for the SNR chart}
    procedure PreparePolarAxes(AChart: TChart; AMax: Double);
    procedure PrepareSatPolarDiagram;
    procedure DrawPolarAxes(AChart: TChart; AMax, ADelta: Double; MeasurementUnit: string);
    procedure GUIsetCaptionsAndHints;
    procedure AddToChart(const Time: TDateTime; value: single);

    procedure InvisibleNFZ;
    procedure VisibleNFZ;
    procedure EnableNFZ;
    function CheckLicenseFiles(const SerialNumber: string=''): boolean;
  public
    procedure ReadMessage_BC(var msg: TMAVmessage);
    procedure ReadGUIMessages(msg: TMAVmessage);
    procedure RecordMessage(msg: TMAVmessage; list: TStringList; LengthFixPart: byte);
    procedure ActAsGUI(var msg: TMAVmessage; list: TStringList);
    procedure NumberMessagesInStatusBar;

  end;

  type TLicBytes =  array [0..18] of byte;

  {$I H480GUI_en.inc}
  {.$I H480GUI_de.inc}

var
  Form1: TForm1;
  UART: TBlockSerial;
  UARTConnected, SerialNumberFound: boolean;
  SensorStream: TMemoryStream;

  starttime: UInt64;
  boottime: TDateTime;
  SequNumberTransmit: byte;
  MessagesSent, MessagesReceived: integer;
  licbytes: TLicBytes;

const
  AppVersion='V1.5 2025-06-07';
  linkLazarus='https://www.lazarus-ide.org/';

  tab1=' ';
  tab2='  ';
  licfilemask='*.pfx';

  maxPorts=10;
  timeout=100;
  defaultbaud=115200;
  wait=5;
  highRPM=100;

  AccMin=850;          {Threshold for accelerometer magnitude (z-axis problem}

  clSatUsed=clGreen;
  clSatVisible=$004080FF;
  clPolarLabel=clSkyBlue;
  clDataSerie1=clRed;
  clSensorOK=clMoneyGreen;
  clSensorMiss=$000065FF;
  clAttention=$008080F0;                           {Farbe Achtung}
  clError=clRed;

  PolarSatSize=8;

{$IFDEF WINDOWS}
  default_port='COM6';
{$ELSE}                                                {UNIX like OS}
  default_port='/dev/ttyACM0';
{$ENDIF}


implementation

{$R *.lfm}

{ TForm1 }

procedure TForm1.InvisibleNFZ;                         {Default}
begin
  btnEnableNFZ.Visible:=false;
  btnDisableNFZ.Visible:=false;
  licbytes:=Default(TLicBytes);
end;

procedure TForm1.VisibleNFZ;                           {If any license file is availbale}
begin
  btnEnableNFZ.Visible:=true;
  btnDisableNFZ.Visible:=true;
  btnEnableNFZ.Enabled:=false;
  btnDisableNFZ.Enabled:=false;
end;

procedure TForm1.EnableNFZ;                            {If a license file matching FC serial number}
begin
  btnEnableNFZ.Visible:=true;
  btnDisableNFZ.Visible:=true;
  btnEnableNFZ.Enabled:=true;
  btnDisableNFZ.Enabled:=true;
end;

function ValidateLicFileName(fn: string): boolean;
const
  serialchars=['0'..'9', 'A'..'F', '-'];

var
  i: integer;

begin
  if length(fn)<>26 then
    exit(false);
  for i:=1 to length(fn) do
    if not (fn[i] in serialchars) then
      exit(false);
  result:=(fn[9]='-') and (fn[18]='-');
end;

function TForm1.CheckLicenseFiles(const SerialNumber: string=''): boolean;
var
  i: integer;
  filelist: TStringList;
  filenameshort: string;
  licstream: TMemoryStream;

begin
  result:=false;
  InvisibleNFZ;
  filelist:=TStringList.Create;
  try
    FindAllFiles(filelist, GetUserDir, licfilemask, false);
    if filelist.Count>0 then begin
      for i:=0 to filelist.Count-1 do begin
        filenameshort:=Uppercase(ChangeFileExt(ExtractFileName(filelist[i]), ''));
        if ValidateLicFileName(filenameshort) then begin
          VisibleNFZ;
          if SerialNumber<>'' then begin
            if filenameshort=UpperCase(SerialNumber) then begin
              licstream:=TMemoryStream.Create;
              try
                licstream.LoadFromFile(filelist[i]);
                licbytes:=Default(TLicBytes);
                if licstream.Size=16 then
                  licstream.Read(licbytes, 16)
                else
                  exit;
                result:=true;
                EnableNFZ;
              finally
                licstream.Free;
              end;
              break;
            end;
          end;
        end;
      end;
    end;
  finally
    filelist.Free;
  end;
end;

procedure TForm1.FormCreate(Sender: TObject);
begin
  UARTconnected:=false;
  GUItext.Text:='';
  GUIsetCaptionsAndHints;
  WriteGUIvalueListHeader;
  ResetSensorsStatus;
  PrepareSatSNRBarChart(capSatSNR+' [db]');
  PrepareSatPolarDiagram;
  PreparePolarAxes(SatPolar, 90);
  BarSatSNR.Clear;
  SatPolarSeries.Clear;
  AddValueLineSeries1.Clear;
  shapeUsed.Pen.Color:=clSatUsed;
  shapeNotUsed.Pen.Color:=clSatVisible;
  CheckLicenseFiles;
 end;

procedure TForm1.GUIsetCaptionsAndHints;
begin
  Caption:=Application.Title+tab2+AppVersion;
  cbSpeed.Text:=IntToStr(defaultbaud);
  cbSpeed.Hint:=hntSpeed;
  cbPort.Hint:=hntPort;
  acConnect.Caption:=capConnect;
  acConnect.Hint:=hntConnect;
  acDisConnect.Caption:=capDisConnect;
  acDisConnect.Hint:=hntDisConnect;
  acClose.Caption:=capClose;
  acSaveGUItext.Caption:=capSaveProt;
  acSaveGUItext.Hint:=hntSaveProt;
  acCopySerial.Caption:=capCopySerial;
  acCopySerial.Hint:=hntCopySerial;
  acDeleteText.Caption:=capDeleteGUItext;
  acDeleteText.Hint:=hntDeleteGUItext;
  acLoadLicense.Caption:=capLicense;
  acLoadLicense.Hint:=hntLicense;

  acSaveGUItext.Caption:=capSaveProt;
  acSaveGUItext.Hint:=hntSaveProt;
  acEnabletesting.Caption:=capEnableTesting;

  lblOtherSats.Caption:='';
  lblFCtimeGPS.Caption:=capFCtime;
  lblFCtimeGPS.Hint:=hntFCtime;
  lblFCtime.Caption:=capFCtime;
  lblFCtime.Hint:=hntFCtime;
  lbIGPSsats.Hint:=hntGPS;
  lbIGLONASSsats.Hint:=hntGLONASS;
  lbISBASsats.Hint:=hntSBAS;
  lblSystime.Caption:=capSystemTime;
  lblSysTime.Hint:=hntTime;
  lblNotUsed.Caption:=capNotUsed;
  lblNotUsed.Hint:=hntNotUsed;
  lblSatUsed.Caption:=capSatUsed;
  lblSatUsed.Hint:=hntSatUsed;
  lblEnableTesting.Caption:=hntEnableTesting;

  SatPolar.Hint:=hntPolar;
  ChartSatSNR.Hint:=hntSatSNR;
  btnClose.Hint:=hntClose;
  StatusBar1.Hint:=hntStatusBar;
  gbMotors.Hint:=hntEnableTesting;

  gbGeoFence.Caption:=capGeoFence;
  gbGeoFence.Hint:=hntGeoFence;
  lblGeoFenceVal.Hint:=hntGeoFenceDist;
  speGeoFence.Hint:=hntGeoFenceDist;
  lblGeoFence.Hint:=hntGeoFenceDist;
  lblHeightLimitVal.Hint:=hntHeightLimit;
  speHeightLimit.Hint:=hntHeightLimit;
  lblHeightLimit.Hint:=hntHeightLimit;
  lblCurrentValue.Caption:=capCurrentValue;
  lblNewValue.Caption:=capNewValue;

  cbLimitMsg.Caption:=capLimitMsg;
  cbHighRPM.Caption:=capHighRPM;
  cbHighRPM.Hint:=hntHighRPM;
  cbRecord.Caption:=capRecord;
  cbRecord.Hint:=hntRecord;
  cbSensor.Caption:=capSensor;
  cbSensor.Hint:=hntSensor;
  chAddValue.Hint:=capAddValue;
  rgAddValue.Caption:=capAddValue;
  rgAddValue.Hint:=hntAddValue;

  tsSensorInfo.Caption:=capSensorInfo;
  tsGPSinfo.Caption:=capGPSinfo;
  tsSettings.Caption:=capSettings;
end;

procedure TForm1.GUItextMouseWheelDown(Sender: TObject; Shift: TShiftState;
  MousePos: TPoint; var Handled: Boolean);
begin
  if ssCtrl in Shift then
    GUItext.Font.Size:=GUItext.Font.Size-1;
end;

procedure TForm1.GUItextMouseWheelUp(Sender: TObject; Shift: TShiftState;
  MousePos: TPoint; var Handled: Boolean);
begin
  if ssCtrl in Shift then
    GUItext.Font.Size:=GUItext.Font.Size+1;
end;

procedure TForm1.Image1Click(Sender: TObject);
begin
  OpenURL(linkLazarus);
end;

function SendUARTMessage(const msg: TMAVmessage; LengthFixPart: byte): boolean;
begin
  result:=false;
  if msg.valid then begin
    if UART.SendBuffer(@msg.msgbytes, msg.msglength+LengthFixPart+2)>LengthFixPart then begin
      result:=true;
      inc(MessagesSent);
    end;
  end;
  sleep(wait);
end;

procedure TForm1.GridPrepare(var grid: TStringGrid; const NumRows: byte);
var
  i: byte;

begin
  grid.RowCount:=NumRows;
  for i:=1 to NumRows-1 do
    grid.Cells[1, i]:='';
end;

procedure SetXYZ(var vle: TValueListEditor; const MeasurementUnit: shortstring='');
begin
  vle.Cells[0, 1]:='X '+MeasurementUnit;
  vle.Cells[0, 2]:='Y '+MeasurementUnit;
  vle.Cells[0, 3]:='Z '+MeasurementUnit;
end;

procedure TForm1.WriteGUIvalueListHeader;
begin
  vlePosition.Cells[0, 1]:='Latitude';
  vlePosition.Cells[0, 2]:='Longitude';
  vlePosition.Cells[0, 3]:='Altitude MSL';
  vlePosition.Cells[0, 4]:='Altitude rel';
  vlePosition.Cells[0, 5]:='Sats visible';
  vlePosition.Cells[0, 6]:='Sats used';
  vlePosition.Cells[0, 7]:='HDOP';
  vlePosition.Cells[0, 8]:='VDOP';
  vlePosition.Cells[0, 9]:='Fix type';

  vleVelocity.Cells[0, 1]:='Velocity';
  vleVelocity.Cells[0, 2]:='Vx';
  vleVelocity.Cells[0, 3]:='Vy';
  vleVelocity.Cells[0, 4]:='Vz';
  vleVelocity.Cells[0, 5]:='Variance';

  vleSystem.Cells[0, 1]:='Vehicle type';
  vleSystem.Cells[0, 2]:='Vehicle ID';
  vleSystem.Cells[0, 3]:='FW version';
  vleSystem.Cells[0, 4]:='FW date';
  vleSystem.Cells[0, 5]:='Real Sense';

  vleSysStatus.Cells[0, 1]:='Sensors present';
  vleSysStatus.Cells[0, 2]:='Sensors enabled';
  vleSysStatus.Cells[0, 3]:='Sensors healty';
  vleSysStatus.Cells[0, 4]:='Drop rate';
  vleSysStatus.Cells[0, 5]:='Comm erros';
  vleSysStatus.Cells[0, 6]:='Error count';
  vleSysStatus.Cells[0, 7]:='EKF status';
  vleSysStatus.Cells[0, 8]:='Voltage';
  vleSysStatus.Cells[0, 9]:='Radio SR24';

  vleBaro.Cells[0, 1]:='Pressure';
  vleBaro.Cells[0, 2]:='Temperature';
  vleBaro.Cells[0, 3]:='Height estimate';

  SetXYZ(vleGyro);
  vleGyro.Cells[0, 4]:='Gyro cali X';
  vleGyro.Cells[0, 5]:='Gyro cali Y';
  vleGyro.Cells[0, 6]:='Gyro cali Z';
  vleGyro.Cells[0, 7]:='IMU temperature';

  SetXYZ(vleAcc, '[mG]');
  vleAcc.Cells[0, 4]:='Magnitude [mG]';
  vleAcc.Cells[0, 5]:='Acc cali X';
  vleAcc.Cells[0, 6]:='Acc cali Y';
  vleAcc.Cells[0, 7]:='Acc cali Z';

  SetXYZ(vleMag);
  vleMag.Cells[0, 4]:='Compass variance';
  vleMag.Cells[0, 5]:='Mag offset X';
  vleMag.Cells[0, 6]:='Mag offset Y';
  vleMag.Cells[0, 7]:='Mag offset Z';

  vleOrientation.Cells[0, 1]:='Roll';
  vleOrientation.Cells[0, 2]:='Pitch';
  vleOrientation.Cells[0, 3]:='Yaw';

  ResetSensorsStatus;
end;

function GetContrastTextColor(const BackColor: TColor): TColor;
begin                                              {Textfaebe abh. vom Hintergrund}
  if (Red(BackColor) * 0.25+
      Green(BackColor) * 0.625+
      Blue(BackColor) * 0.125) > 90 then
    result := clBlack
  else
    result := clWhite;
end;

procedure CellColorSetting(aGrid: TValueListEditor; Farbe: TColor); {Zellen einfärben}
begin
  aGrid.Canvas.Brush.Color:=Farbe;
  aGrid.Canvas.Font.Color:=GetContrastTextColor(Farbe);
end;

procedure TForm1.StopAllTimer;
begin
  timerGUI.Enabled:=false;
  timerSensors.Enabled:=false;
end;

procedure TForm1.ResetSensorsStatus;
begin
  gbPosition.Color:=clSensorMiss;
  gbBaro.Color:=clSensorMiss;
  gbAcc.Color:=clSensorMiss;
  gbGyro.Color:=clSensorMiss;
  gbMag.Color:=clSensorMiss;
  gbSysStatus.Color:=clSensorMiss;
  shapeGPSOK.Pen.Color:=clSensorMiss;
  shapeIMUOK.Pen.Color:=clSensorMiss;
  shapeSonar.Pen.Color:=clSensorMiss;
  shapeRSOK.Pen.Color:=clSensorMiss;
  shapeESCOK.Pen.Color:=clSensorMiss;
  picGPS.ImageIndex:=11;
  picGLONASS.ImageIndex:=11;
  picSBAS.ImageIndex:=11;
  lblOtherSats.Caption:='';
  vleSysStatus.Cells[1, 9]:='';
end;

procedure WriteCSVRawHeader(var list: TStringList);
var
  s: string;
  i: integer;

begin
  list.Clear;
  s:=rsTime;
  for i:=0 to 105 do
    s:=s+';'+Format('%.*d', [2, i]);
  list.Add(s);
end;

procedure SetStartValuesForGlobelVariables;
begin
  SequNumberTransmit:=0;
  MessagesSent:=0;
  MessagesReceived:=0;
  SerialNumberFound:=false;
  starttime:=GetTickCount64;
  boottime:=0;
end;

function ConnectUART(port, speed: string): string;
begin
  result:='';
  if UARTconnected then
    exit;
  UART:=TBlockSerial.Create;
  SensorStream:=TMemoryStream.Create;
  {$IFDEF LINUX}
    UART.LinuxLock:=false;
  {$ENDIF}
  UART.Connect(port);
  sleep(200);
  UART.Config(StrToIntDef(speed, defaultbaud), 8, 'N', SB1, false, false); {Config default 115200 baud, 8N1}
  if UART.LastError=0 then begin
    UARTConnected:=true;
    result:='Status: '+UART.LastErrorDesc;
  end else begin
    result:='Error: '+UART.LastErrorDesc;
  end;
end;

procedure DisconnectUART;
begin
  if UARTConnected then begin
    try
      UART.CloseSocket;
    finally
      UART.Free;
      SensorStream.Free;
      UARTConnected:=false;
    end;
  end;
end;

procedure TForm1.NumberMessagesInStatusBar;
begin
  StatusBar1.Panels[0].Text:='S: '+IntToStr(MessagesSent);
  StatusBar1.Panels[1].Text:='R: '+IntToStr(MessagesReceived);
end;

procedure TForm1.acConnectExecute(Sender: TObject);
var
  msg: TMAVmessage;
  csvlist: TStringList;

begin
  csvlist:=TStringList.Create;
  try
    msg:=Default(TMAVmessage);
    SetStartValuesForGlobelVariables;
    ResetSensorsStatus;
    StatusBar1.Panels[0].Text:='0';                    {Sent messages}
    StatusBar1.Panels[1].Text:='0';                    {Received messages}
    WriteCSVRawHeader(csvlist);
    StatusBar1.Panels[2].Text:=ConnectUART(cbPort.Text, cbSpeed.Text);
    If UARTconnected then begin
      StatusBar1.Panels[2].Text:=StatusBar1.Panels[2].Text+'  -  '+rsConnected;
      btnGeoFence.Enabled:=UARTConnected;
      btnHeightLimit.Enabled:=UARTConnected;
      ActAsGUI(msg, csvlist);
      NumberMessagesInStatusBar;
      SaveDialog1.FilterIndex:=1;
      SaveDialog1.FileName:='BCmessages_'+FormatDateTime('yyyymmdd_hhnnss', now)+'.csv';
      if cbRecord.Checked and (csvlist.Count>1) and SaveDialog1.Execute then begin
        csvlist.SaveToFile(SaveDialog1.FileName);
        StatusBar1.Panels[2].Text:=SaveDialog1.FileName+rsSaved;
      end;
    end;
  finally
    csvlist.Free;
  end;
end;

procedure TForm1.acCopySerialExecute(Sender: TObject);
begin
  if vleSystem.Cells[1, 2]<>'' then
    Clipboard.AsText:=vleSystem.Cells[1, 2];
end;

procedure TForm1.acDeleteTextExecute(Sender: TObject);
begin
  GUItext.Text:='';
end;

procedure TForm1.acCloseExecute(Sender: TObject);
begin
  Close;
end;

procedure TForm1.RecordMessage(msg: TMAVmessage; list: TStringList; LengthFixPart: byte);
var
  s: string;
  i: integer;

begin
  s:=FormatFloat(floatformat3, (GetTickCount64-starttime)/1000);
  for i:=0 to msg.msglength+LengthFixPart+1 do begin
    s:=s+';'+IntToHex(msg.msgbytes[i], 2);
  end;
  list.Add(s);
end;

procedure SendGUIParamRequest;
var
  msg: TMAVmessage;

begin
  CreateGUI_PARAM_REQUEST_LIST(msg);
  SendUARTMessage(msg, LengthFixPartBC);
  CreateGUI_MISSION_REQUEST_INT(msg, 1);
  SendUARTMessage(msg, LengthFixPartBC);
  CreateGUI_MISSION_REQUEST_INT(msg, 11);
  SendUARTMessage(msg, LengthFixPartBC);
  CreateGUI_MISSION_REQUEST_INT(msg, 12);
  SendUARTMessage(msg, LengthFixPartBC);
end;

procedure TForm1.FillGUIPosition24(const sats: TGPSdata);
begin
  vlePosition.Cells[1, 1]:=FormatCoordinates(sats.lat);
  vlePosition.Cells[1, 2]:=FormatCoordinates(sats.lon);
  vlePosition.Cells[1, 3]:=FormatAltitude(sats.altMSL);
  vlePosition.Cells[1, 6]:=IntToStr(sats.sats_inuse);
  vlePosition.Cells[1, 7]:=FormatDOP(sats.eph);
  vlePosition.Cells[1, 8]:=FormatDOP(sats.epv);
  vlePosition.Cells[1, 9]:=FixTypeToStr(sats.fix_type);

  vleVelocity.Cells[1, 1]:=FormatSpeed(sats.vel);

  case rgAddValue.ItemIndex of
    6: AddToChart(boottime, sats.altMSL/1000);
    8: AddToChart(boottime, sats.eph/100);
    9: AddToChart(boottime, sats.epv/100);
  end;
end;

{Sat ID are PRN numbers.
 See https://continuouswave.com/forum/viewtopic.php?t=1696
 SBAS: https://gssc.esa.int/navipedia/index.php/SBAS_Fundamentals}

procedure TForm1.FillGPS_STATUS(const sats: TGPSdata);
begin
  if sats.numGPS_visible>0 then
    picGPS.ImageIndex:=10
  else
    picGPS.ImageIndex:=11;
  if sats.numGLONASS_visible>0 then
    picGLONASS.ImageIndex:=10
  else
    picGLONASS.ImageIndex:=11;
  if sats.numSBAS_visible>0 then
    picSBAS.ImageIndex:=10
  else
    picSBAS.ImageIndex:=11;
  if sats.numOther_visible>0 then
    lblOtherSats.Caption:='Other Sat-PRNs: '+IntToStr(sats.numOther_visible)
  else
    lblOtherSats.Caption:='';

  vlePosition.Cells[1, 5]:=IntToStr(sats.sats_visible);
  if (gbPosition.Color<>clSensorOK) and (gbPosition.Color<>clSatUsed) then
    gbPosition.Color:=clSensorOK;
  if shapeGPSOK.Pen.Color<>clSensorOK then
    shapeGPSOK.Pen.Color:=clSensorOK;
end;

procedure TForm1.FillGUIPosition33(const sats: TGPSdata);
begin
  vlePosition.Cells[1, 1]:=FormatCoordinates(sats.lat);
  vlePosition.Cells[1, 2]:=FormatCoordinates(sats.lon);
  vlePosition.Cells[1, 4]:=FormatAltitude(sats.alt_rel);
  vleBaro.Cells[1, 3]:=FormatAltitude(sats.alt_rel);

  if rgAddValue.ItemIndex=7 then
    AddToChart(boottime, sats.alt_rel/1000);

  vleVelocity.Cells[1, 2]:=FormatXYZSpeed(sats.vx);
  vleVelocity.Cells[1, 3]:=FormatXYZSpeed(sats.vy);
  vleVelocity.Cells[1, 4]:=FormatXYZSpeed(sats.vz);
end;

procedure TForm1.FillGUIIMU(const data: THWstatusData);
var
  Magnitude: double;

begin
  vleAcc.Cells[1, 1]:=IntToStr(data.AccX);
  vleAcc.Cells[1, 2]:=IntToStr(data.AccY);
  vleAcc.Cells[1, 3]:=IntToStr(data.AccZ);
  Magnitude:=Value3D(data.AccX, data.AccY, data.AccZ);
  vleAcc.Cells[1, 4]:=FormatFloat(floatformat2, Magnitude);
  if rgAddValue.ItemIndex=5 then
    AddToChart(boottime, Magnitude);

  vleGyro.Cells[1, 1]:=IntToStr(data.GyroX);
  vleGyro.Cells[1, 2]:=IntToStr(data.GyroY);
  vleGyro.Cells[1, 3]:=IntToStr(data.GyroZ);

  vleMag.Cells[1, 1]:=IntToStr(data.MagX);
  vleMag.Cells[1, 2]:=IntToStr(data.MagY);
  vleMag.Cells[1, 3]:=IntToStr(data.MagZ);
end;

procedure TForm1.vleAccPrepareCanvas(Sender: TObject; aCol, aRow: Integer;
  aState: TGridDrawState);
begin
  if aCol=1 then begin
    if (aRow=4) and (vleAcc.Cells[1, 4]<>'') and
       (StrToFloat(vleAcc.Cells[1, 4])<AccMin)  then
      CellColorSetting(vleAcc,clSensorMiss);

  end;
end;

// below 0.5 is good, 0.5~0.79 is warning, 0.8 or higher is bad)
procedure TForm1.vleVelocityPrepareCanvas(Sender: TObject; aCol, aRow: Integer;
  aState: TGridDrawState);
var
  velocity: single;

begin
  if aCol=1 then begin
    if (aRow=5) and (vleAcc.Cells[1, 5]<>'') then begin
      velocity:=StrToFloat(vleAcc.Cells[1, 5]);
      if velocity>=0.8 then
        CellColorSetting(vleAcc,clError)
      else
        if velocity>=0.5 then
          CellColorSetting(vleAcc,clAttention);
    end;
  end;
end;

procedure TForm1.FillSENSOR_OFFSETS(const data: THWstatusData);
begin
  vleAcc.Cells[1, 5]:=FormatFloat(floatformat3, data.AccCaliX);
  vleAcc.Cells[1, 6]:=FormatFloat(floatformat3, data.AccCaliY);
  vleAcc.Cells[1, 7]:=FormatFloat(floatformat3, data.AccCaliZ);

  vleGyro.Cells[1, 4]:=FormatFloat(floatformat3, data.GyroCaliX);
  vleGyro.Cells[1, 5]:=FormatFloat(floatformat3, data.GyroCaliY);
  vleGyro.Cells[1, 6]:=FormatFloat(floatformat3, data.GyroCaliZ);

  vleMag.Cells[1, 5]:=IntToStr(data.MagOfsX);
  vleMag.Cells[1, 6]:=IntToStr(data.MagOfsY);
  vleMag.Cells[1, 7]:=IntToStr(data.MagOfsZ);
end;

procedure TForm1.FillAttitude(data: TAttitudeData);
begin
  vleOrientation.Cells[1, 1]:=FormatFloat(floatformat1, data.roll);
  vleOrientation.Cells[1, 2]:=FormatFloat(floatformat1, data.pitch);
  vleOrientation.Cells[1, 3]:=FormatFloat(floatformat1, data.yaw);
  case rgAddValue.ItemIndex of
    2: AddToChart(boottime, data.roll);
    3: AddToChart(boottime, data.pitch);
    4: AddToChart(boottime, data.yaw);
  end;
end;

{ EKF status
        Value	Name	Description
 1       1	ESTIMATOR_ATTITUDE	        True if the attitude estimate is good
 0       2	ESTIMATOR_VELOCITY_HORIZ	True if the horizontal velocity estimate is good
 1       4	ESTIMATOR_VELOCITY_VERT	        True if the vertical velocity estimate is good
 0       8	ESTIMATOR_POS_HORIZ_REL         True if the horizontal position (relative) estimate is good

 0       16	ESTIMATOR_POS_HORIZ_ABS	        True if the horizontal position (absolute) estimate is good
 1       32	ESTIMATOR_POS_VERT_ABS	        True if the vertical position (absolute) estimate is good
 0       64	ESTIMATOR_POS_VERT_AGL	        True if the vertical position (above ground) estimate is good
 1       128	ESTIMATOR_CONST_POS_MODE	True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)

 1       256	ESTIMATOR_PRED_POS_HORIZ_REL	True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
 1       512	ESTIMATOR_PRED_POS_HORIZ_ABS	True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
 0       1024	ESTIMATOR_GPS_GLITCH	        True if the EKF has detected a GPS glitch
 0       2048	ESTIMATOR_ACCEL_ERROR	        True if the EKF has detected bad accelerometer data
}

procedure TForm1.FillEKF_STATUS_REPORT(data: TAttitudeData);
begin
  vleVelocity.Cells[1, 5]:=FormatFloat(floatformat3, data.velocity_variance);
  vleMag.Cells[1, 4]:=FormatFloat(floatformat3, data.compass_variance);
  vleSysStatus.Cells[1, 7]:=IntToHex(data.EKFstatus, 4);
end;

{ YTH OK w/o RS: 00 A0 FC 2F      with RS: 02 A0 FC 6F

Flags from low to high:

1    MAV_SYS_STATUS_SENSOR_3D_GYRO=1                     0x01 3D gyro
1    MAV_SYS_STATUS_SENSOR_3D_ACCEL=2                    0x02 3D accelerometer
1    MAV_SYS_STATUS_SENSOR_3D_MAG=4                      0x04 3D magnetometer
1    MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE=8           0x08 absolute pressure

0    MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE=16      0x10 differential pressure
1    MAV_SYS_STATUS_SENSOR_GPS=32                        0x20 GPS
0(1) MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW=64               0x40 optical flow
0    MAV_SYS_STATUS_SENSOR_VISION_POSITION=128           0x80 computer vision position

0    MAV_SYS_STATUS_SENSOR_LASER_POSITION=256            0x100 laser based position
0    MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH=512     0x200 external ground truth (Vicon or Leica)
1    MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL=1024     0x400 3D angular rate control
1    MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION=2048   0x800 attitude stabilization

1    MAV_SYS_STATUS_SENSOR_YAW_POSITION=4096             0x1000 yaw position
1    MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL=8192       0x2000 z/altitude control
1    MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL=16384     0x4000 x/y position control
1    MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS=32768           0x8000 motor outputs / control

0(1) MAV_SYS_STATUS_SENSOR_RC_RECEIVER=65536             0x10000 Radio SR24
0    MAV_SYS_STATUS_SENSOR_3D_GYRO2=131072               0x20000 2nd 3D gyro
0    MAV_SYS_STATUS_SENSOR_3D_ACCEL2=262144              0x40000 2nd 3D accelerometer
0    MAV_SYS_STATUS_SENSOR_3D_MAG2=524288                0x80000 2nd 3D magnetometer

0    MAV_SYS_STATUS_GEOFENCE=1048576                     0x100000 geofence
1    MAV_SYS_STATUS_AHRS=2097152                         0x200000 AHRS subsystem health (Attitude Heading Reference System)
0    MAV_SYS_STATUS_TERRAIN=4194304                      0x400000 Terrain subsystem health
1    MAV_SYS_STATUS_REVERSE_MOTOR=8388608                0x800000 Motors are reversed   Sonar !

0    MAV_SYS_STATUS_LOGGING=16777216                     0x1000000 Logging
0(1) MAV_SYS_STATUS_SENSOR_BATTERY=33554432              0x2000000 Battery
0    MAV_SYS_STATUS_SENSOR_PROXIMITY=67108864            0x4000000 Proximity
0    MAV_SYS_STATUS_SENSOR_SATCOM=134217728              0x8000000 Satellite Communication

0    MAV_SYS_STATUS_PREARM_CHECK=268435456               0x10000000 pre-arm check status. Always healthy when armed
0    MAV_SYS_STATUS_OBSTACLE_AVOIDANCE=536870912         0x20000000 Avoidance/collision prevention
0    MAV_SYS_STATUS_SENSOR_PROPULSION=1073741824         0x40000000 propulsion (actuator, esc, motor or propellor)
0    MAV_SYS_STATUS_EXTENSION_USED=2147483648            0x80000000 Extended bit-field are used for further sensor status bits (needs to be set in onboard_control_sensors_present only)
}

procedure TForm1.FillGUI_SYS_STATUS(const msg: TMAVmessage; var data: TGPSdata);
var
  err: integer;
  healty, enabld: UInt32;

begin
  vleSysStatus.Cells[1, 1]:=IntToHexSpace(MavGetUInt32(msg, LengthFixPartBC));
  enabld:=MavGetUInt32(msg, LengthFixPartBC+4);
  vleSysStatus.Cells[1, 2]:=IntToHexSpace(enabld);
  healty:=MavGetUInt32(msg, LengthFixPartBC+8);
  vleSysStatus.Cells[1, 3]:=IntToHexSpace(healty);

  vleSysStatus.Cells[1, 4]:=IntToStr(MavGetUInt16(msg, LengthFixPartBC+18));
  vleSysStatus.Cells[1, 5]:=IntToStr(MavGetUInt16(msg, LengthFixPartBC+20));
  err:=MavGetUInt16(msg, LengthFixPartBC+22)+MavGetUInt16(msg, LengthFixPartBC+24)+
       MavGetUInt16(msg, LengthFixPartBC+26)+MavGetUInt16(msg, LengthFixPartBC+28);
  vleSysStatus.Cells[1, 6]:=IntToStr(err);
  vleSysStatus.Cells[1, 8]:=FormatFloat(floatformat2, data.voltage/1000)+'V';
  if data.batt_cap<max8 then
    vleSysStatus.Cells[1, 8]:=vleSysStatus.Cells[1, 8]+'  ('+IntToStr(data.batt_cap)+'%)';

  if (enabld and $10000)=$10000 then                   {Radio SR24}
    vleSysStatus.Cells[1, 9]:=rsEnabled;
  if (healty and $10000)=$10000 then
    vleSysStatus.Cells[1, 9]:=rsConnected;

  if (enabld and $40)=$40 then
    vleSystem.Cells[1, 5]:=rsAvailable
  else
    vleSystem.Cells[1, 5]:=rsNotMounted;
  if (healty and $2000040)=$2000040 then begin         {Real Sense}
    if shapeRSOK.Pen.Color<>clSensorOK then
      shapeRSOK.Pen.Color:=clSensorOK;
  end;

  if (healty and 1)=1 then begin
    if gbGyro.Color<>clSensorOK then
      gbGyro.Color:=clSensorOK;
  end else begin
    data.sensors_OK:=false;
    gbGyro.Color:=clSensorMiss;
  end;
  if (healty and 2)=2 then begin
    if gbAcc.Color<>clSensorOK then
      gbAcc.Color:=clSensorOK;
  end else begin
    data.sensors_OK:=false;
    gbAcc.Color:=clSensorMiss;
  end;
  if (healty and 4)=4 then begin
    if gbMag.Color<>clSensorOK then
      gbMag.Color:=clSensorOK;
  end else begin
    data.sensors_OK:=false;
    gbMag.Color:=clSensorMiss;
  end;
  if (healty and $8000)=$8000 then begin
    if shapeESCOK.Pen.Color<>clSensorOK then
      shapeESCOK.Pen.Color:=clSensorOK;
  end else begin
    shapeESCOK.Pen.Color:=clSensorMiss;
  end;
  if (healty and $0F)=$0F then begin                   {Gyro, Acc, Mag and Baro}
    if shapeIMUOK.Pen.Color<>clSensorOK then
      shapeIMUOK.Pen.Color:=clSensorOK;
  end else begin
    data.sensors_OK:=false;
    shapeIMUOK.Pen.Color:=clSensorMiss;
  end;

  if (healty and 8)=8 then begin
    if gbBaro.Color<>clSensorOK then
      gbBaro.Color:=clSensorOK;
  end else begin
    data.sensors_OK:=false;
    gbBaro.Color:=clSensorMiss;
  end;
  if (healty and $800000)=$800000 then begin
    if shapeSonar.Pen.Color<>clSensorOK then
      shapeSonar.Pen.Color:=clSensorOK;
  end else begin
    if vleSystem.Cells[1, 1]='TyphoonH' then           {Sonar only for Typhoon H}
      data.sensors_OK:=false;
    shapeSonar.Pen.Color:=clSensorMiss;
  end;

  if data.sensors_OK and (err=0) then
    gbSysStatus.Color:=clSensorOK
  else
    gbSysStatus.Color:=clSensorMiss;
end;

procedure TForm1.ClearGUI;
var
  i: byte;

begin
  SerialNumberFound:=false;
  btnTurnAll.Enabled:=false;
  BarSatSNR.Clear;
  SatPolarSeries.Clear;
  ResetSensorsStatus;
  AddValueLineSeries1.Clear;
  lblSysTime.Caption:=rsTimeUTC;
  GUItext.Text:='';
  acCopySerial.Enabled:=false;
  for i:=1 to vleSystem.RowCount-1 do
    vleSystem.Cells[1, i]:='';
end;

function FormatBootTime(const data: TGPSdata): string;
begin
  result:=FormatDateTime(timezzz, data.boottime);
end;

procedure TForm1.AddToChart(const Time: TDateTime; value: single);
begin
  chAddValue.BeginUpdateBounds;
  if time>0 then begin
    if AddValueLineSeries1.Count>1000 then
      AddValueLineSeries1.Delete(0);
    AddValueLineSeries1.AddXY(time, value);
  end;
  chAddValue.EndUpdateBounds;
end;

{https://mavlink.io/en/messages/common.html}
procedure TForm1.ReadGUIMessages(msg: TMAVmessage);
var
  GUI_GPSdata: TGPSdata;
  Sensors: THWstatusData;
  DronePos: TAttitudeData;
  Values24: TData96;

  s, dt: string;
  value: single;

(*
  procedure TestDATA96(v: TData96);      {Test DATA96}
  var
    i: byte;
    s: string;

  begin
    s:=lblFCtime.Caption;
    for i:=0 to 23 do begin
      s:=s+';'+FormatFloat('0,000', v[i]);
      if i=1 then          {Selection of the value you want to see in the chart; 0 is IMU temperature}
        AddToChart(boottime, v[i]);
    end;
    GUItext.Lines.Add(s);
  end;
*)

begin
//  GPSdata_SetDefaultValues(GUI_GPSdata);
  GUI_GPSdata:=Default(TGPSdata);
  Sensors:=Default(THWstatusData);
  DronePos:=Default(TAttitudeData);
  Values24:=Default(TData96);

//  rgAddValue.ItemIndex:=-1;      {Test DATA96}

  case msg.msgid of
    0: begin
      SendGUIParamRequest;
      lblOK.Caption:=tab1;
    end;
    1: begin
      SYS_STATUS(msg, LengthFixPartBC, GUI_GPSdata);
      FillGUI_SYS_STATUS(msg, GUI_GPSdata);
    end;
    2: begin
      SYS_TIME(msg, LengthFixPartBC, GUI_GPSdata);
      lblSysTime.Caption:=FormatDateTime(timefull, GUI_GPSdata.timeUTC);
    end;

    22: begin
      s:=PARAM_VALUE(msg, LengthFixPartBC, value);
//      GUItext.Lines.Add(s+' = '+FormatFloat('0', value)); {Option: Let's look what else may come}
      if s=pGeoFence then
        lblGeoFenceVal.Caption:=FormatFloat('0', value)+'m'
      else
        if s=pHeightLimit then
          lblHeightLimitVal.Caption:=FormatFloat('0', value)+'m';
    end;

    24: begin
      GPS_RAW_INT(msg, LengthFixPartBC, GUI_GPSdata);
      FillGUIposition24(GUI_GPSdata);
      timerSensors.Enabled:=false;
    end;
    25: begin
      GPS_STATUS(msg, LengthFixPartBC, GUI_GPSdata);
      FillGPS_STATUS(GUI_GPSdata);
      CreateSatSNRBarChart(GUI_GPSdata);
      CreateSatPolarDiagram(GUI_GPSdata);
    end;

    27: begin
      RAW_IMU(msg, LengthFixPartBC, sensors);
      FillGUIIMU(sensors);
    end;

    29: begin
      SCALED_PRESSURE(msg, LengthFixPartBC, Sensors);
      vleBaro.Cells[1, 1]:=FormatFloat(floatformat2, Sensors.pressure_abs)+'hPa';
      vleBaro.Cells[1, 2]:=FormatFloat(floatformat2, Sensors.baro_temp/100)+'°C';
      if rgAddvalue.ItemIndex=1 then
        AddToChart(boottime, Sensors.baro_temp/100);
    end;
    30: begin
      ATTITUDE(msg, LengthFixPartBC, DronePos);
      FillAttitude(DronePos);
    end;

    33: begin
      GLOBAL_POSITION_INT(msg, LengthFixPartBC, GUI_GPSdata);
      boottime:=GUI_GPSdata.boottime;
      lblFCtime.Caption:=FormatBootTime(GUI_GPSdata);
      lblFCtimeGPS.Caption:=FormatBootTime(GUI_GPSdata);
      FillGUIposition33(GUI_GPSdata);
    end;

    52: begin
      vleSystem.Cells[1, 1]:=GetSYSTEM(msg, LengthFixPartBC, s, dt);
      vleSystem.Cells[1, 3]:=s;
      vleSystem.Cells[1, 4]:=dt;

    end;

    56: begin
      if not SerialNumberFound then begin
        s:=GetSERIAL(msg, LengthFixPartBC);
        vleSystem.Cells[1, 2]:=s;
        CheckLicenseFiles(s);
        Caption:=Application.Title+tab2+s;
        SerialNumberFound:=true;
        acCopySerial.Enabled:=true;
      end;
    end;
    58: if (msg.msgbytes[6]=1) and (msg.msgbytes[7]=1) and (msg.msgbytes[8]=1) then begin
          lblOK.Caption:='OK';
        end else begin
          lblOK.Caption:='Invalid';
          GUItext.Lines.Add(lblOK.Caption+' license key');
        end;

    150: begin
      if vleGyro.Cells[1, 4]='' then begin
        SENSOR_OFFSETS(msg, LengthFixPartBC, Sensors);
        FillSENSOR_OFFSETS(sensors);
      end;
    end;

    172: begin
      DATA96(msg, LengthFixPartBC, Values24);

//      TestDATA96(Values24);          {Test DATA96}

      vleGyro.Cells[1, 7]:=FormatFloat(floatformat2, Values24.value[0])+'°C';  {IMU temp}
      if rgAddValue.ItemIndex=0 then
        AddToChart(boottime, Values24.value[0]);
    end;

    193: begin
      EKF_STATUS_REPORT(msg, LengthFixPartBC, DronePos);
      FillEKF_STATUS_REPORT(DronePos);
    end;

    253: GUItext.Lines.Add(FormatDateTime(timezzz, boottime)+tab1+STATUSTEXT(msg, LengthFixPartBC, ' '));
  end;
end;

procedure TForm1.ActAsGUI(var msg: TMAVmessage; list: TStringList);
begin
  ClearGUI;
  SerialNumberFound:=false;
  timerGUI.Enabled:=true;
  acEnableTesting.Enabled:=true;
  while (UART.LastError=0) and UARTConnected do begin
    if UART.CanRead(0) then begin
      ReadMessage_BC(msg);
      if msg.valid then begin
        ReadGUIMessages(msg);
        if cbRecord.Checked then
          RecordMessage(msg, list, LengthFixPartBC);
        inc(MessagesReceived);
      end;
    end;
    if cbLimitMsg.Checked and (GUItext.Lines.Count>600) then
      GUItext.Lines.Clear;
    Application.ProcessMessages;
  end;
end;

procedure TForm1.ReadMessage_BC(var msg: TMAVmessage);
var
  b, len: byte;
  i, counter: integer;

begin
  msg.valid:=false;
  counter:=0;
  repeat
    b:=UART.RecvByte(timeout);
    inc(counter);
  until (b=MagicBC) or (UART.LastError<>0) or (not UARTConnected) or (Counter>300);
  msg.msgbytes[0]:=b;
  len:=UART.RecvByte(timeout);
  msg.msgbytes[1]:=len;                                {Message length}
  msg.msglength:=len;
  for i:=2 to len+LengthFixPartBC+1 do
    msg.msgbytes[i]:=UART.RecvByte(timeout);
  if CheckCRC16MAV(msg, LengthFixPartBC) then begin
    msg.sysid:=msg.msgbytes[3];
    msg.targetid:=msg.msgbytes[4];
    msg.msgid:=msg.msgbytes[5];
    msg.valid:=true;
  end;
end;

procedure TForm1.acDisconnectExecute(Sender: TObject);
begin
  StopAllTimer;
  DisconnectUART;
  btnGeoFence.Enabled:=UARTConnected;
  btnHeightLimit.Enabled:=UARTConnected;
  btnDisableNFZ.Enabled:=UARTConnected;
  btnEnableNFZ.Enabled:=UARTConnected;
  acEnableTesting.Enabled:=UARTConnected;
  StatusBar1.Panels[2].Text:=rsDisconnected;
end;

procedure TForm1.acEnableTestingExecute(Sender: TObject);
begin
  btnTurnAll.Enabled:=false;
  if UARTconnected then begin
    if MessageDlg(rsConfirm, msgPropellerRemoved,
                  mtConfirmation, [mbYes, mbNo], 0, mbNo)=mrYes then begin
      btnTurnAll.Enabled:=true;
    end;
  end;
end;

procedure TForm1.acLoadLicenseExecute(Sender: TObject);
begin
  OpenDialog.Title:=capLicense;
  if OpenDialog.Execute then begin
    CopyFile(OpenDialog.FileName, GetUserDir+ExtractFileName(OpenDialog.FileName), true);
    CheckLicenseFiles(vleSystem.Cells[1, 2]);
  end;
end;

procedure TForm1.acSaveGUItextExecute(Sender: TObject);
begin
  SaveDialog1.Title:=hntSaveProt;
  SaveDialog1.FilterIndex:=2;
  SaveDialog1.FileName:='TextMessages_'+FormatDateTime('yyyymmdd_hhnnss', now)+'.txt';
  if SaveDialog1.Execute then begin
    if GUItext.Lines.Count>0 then begin
      GUItext.Lines.SaveToFile(SaveDialog1.FileName);
      StatusBar1.Panels[2].Text:=SaveDialog1.FileName+rsSaved;
      GUItext.Lines.Clear;
    end;
  end;
end;

procedure TForm1.acScanPortsExecute(Sender: TObject);
var
{$IFDEF UNIX}
  cmd: TProcess;
  list: TStringList;
{$ENDIF}
  i: integer;

begin
{$IFDEF WINDOWS}
  cbPort.Text:='';
  cbPort.Items.Clear;
  cbPort.Items.CommaText:=GetSerialPortNames;
  if cbPort.Items.Count>0 then begin
    cbPort.Text:=cbPort.Items[cbPort.Items.Count-1];
    for i:=0 to  cbPort.Items.Count-1 do begin
      GUItext.Lines.Add(cbPort.Items[i]);              {Make for Win same as for LINUX}
    end;
    StatusBar1.Panels[2].Text:=cbPort.Items[cbPort.Items.Count-1];
  end else
    StatusBar1.Panels[2].Text:=errNoUSBport;

{$ENDIF}
{$IFDEF UNIX}
  cmd:=TProcess.Create(nil);
  list:=TStringList.Create;
  try
    cmd.Options:=cmd.Options+[poWaitOnExit, poUsePipes];
    cmd.Executable:='ls';
    for i:=0 to cbPort.Items.count-1 do begin
      cmd.Parameters.Clear;
      cmd.Parameters.Add(cbPort.Items[i]);
      cmd.Execute;
      list.LoadFromStream(cmd.Output);
      if list.Count>0 then begin
        StatusBar1.Panels[2].Text:=list[0];
        GUItext.Lines.Add(list[0]);
      end;
    end;
    if GUItext.Lines.Count<1 then
      StatusBar1.Panels[2].Text:=errNoUSBport;
  finally
    cmd.Free;
    list.Free;
  end;
{$ENDIF}
end;

procedure CreateMsg57(var msg: TMAVmessage; const serial_no: shortstring; cmd: byte);
var
  i: byte;

begin
  CreateStandardGUIMsg(msg, 18);
  msg.msgbytes[5]:=57;
  msg.msgbytes[6]:=cmd;
  for i:=0 to 15 do
    msg.msgbytes[i+LengthFixPartBC+2]:=licbytes[i];
  SetCRC_BC(msg);
end;


procedure TForm1.btnDisableNFZClick(Sender: TObject);
var
  msg: TMAVmessage;

begin
  lblOK.Caption:=tab1;
  if (vleSystem.Cells[1, 2]<>'') and UARTConnected then begin
    CreateMsg57(msg, vleSystem.Cells[1, 2], 2);
    SendUARTMessage(msg, LengthFixPartBC);
  end;
end;

procedure TForm1.btnEnableNFZClick(Sender: TObject);
var
  msg: TMAVmessage;

begin
  lblOK.Caption:=tab1;
  if (vleSystem.Cells[1, 2]<>'') and UARTConnected then begin
    CreateMsg57(msg, vleSystem.Cells[1, 2], 8);
    SendUARTMessage(msg, LengthFixPartBC);
  end;
end;

procedure TForm1.btnGeoFenceClick(Sender: TObject);
var
  msg: TMAVmessage;

begin
  CreateGUI_PARAM_SET(msg, pGeoFence, single(speGeoFence.Value));
  SendUARTMessage(msg, LengthFixPartBC);
end;

procedure TForm1.btnHeightLimitClick(Sender: TObject);
var
  msg: TMAVmessage;

begin
  CreateGUI_PARAM_SET(msg, pHeightLimit, single(speHeightLimit.Value));
  SendUARTMessage(msg, LengthFixPartBC);
end;

procedure TForm1.btnTurnAllClick(Sender: TObject);
var
  MotorCommand: TCommandLong;
  msg: TMAVmessage;

begin
  if UARTconnected then begin
    MotorCommand:=Default(TCommandLong);
    MotorCommand.commandID:=209;
    MotorCommand.params[0]:=255;                       {Motor ID; 255 for all}
    MotorCommand.params[2]:=29;                        {RPM}
    if cbHighRPM.Checked then
      MotorCommand.params[2]:=highRPM;
    MotorCommand.params[3]:=2500;                      {Duration}
    CreateGUI_COMMAND_LONG(msg, motorcommand);
    SendUARTMessage(msg, LengthFixPartBC);
  end;
end;

procedure TForm1.cbPortDblClick(Sender: TObject);
begin
  acScanPortsExecute(self);
end;

procedure TForm1.FormActivate(Sender: TObject);
begin
  if not UARTconnected then begin
    StopAllTimer;
    acScanPortsExecute(self);
    btnConnect.SetFocus;
  end;
end;

procedure TForm1.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  DisconnectUART;
end;

{Up to now I had the impression that the motor numbering is counterclockwise.
 This was also my interpretation of the motor error codes in Motor_status in
 flight logs. Also the error beep code support this view.

The motor numbers in the COMMAND_LONG message clockwise starting with 1 front right.
This means the motor numbers in command message are wrong assigned? }

procedure TForm1.picMotorsMouseDown(Sender: TObject; Button: TMouseButton;
                                    Shift: TShiftState; X, Y: Integer);
var
  hpos, vpos: byte;
  MotorCommand: TCommandLong;
  msg: TMAVmessage;

begin
  MotorCommand:=Default(TCommandLong);
  MotorCommand.commandID:=209;
  MotorCommand.params[2]:=29;                          {RPM}
  if cbHighRPM.Checked then
    MotorCommand.params[2]:=highRPM;
  MotorCommand.params[3]:=1000;                        {Duration}

  if x>240 then vpos:=4 else
    if x>162 then vpos:=3 else
      if x>90 then vpos:=2 else
        vpos:=1;
  if y>170 then hpos:=3 else
    if y>70 then hpos:=2 else
      hpos:=1;
  if hpos=1 then begin
    if (vpos=1) or (vpos=2) then begin
      StatusBar1.Panels[2].Text:=rsMotor+' 1 '+rsSelected+rsACW;
      MotorCommand.params[0]:=6;                       {Numbering CCW used}
    end;
    if (vpos=3) or (vpos=4) then begin
      StatusBar1.Panels[2].Text:=rsMotor+' 6 '+rsSelected+rsBCCW;
      MotorCommand.params[0]:=1;
    end;
  end else begin
    if hpos=2 then begin
      if vpos=1 then begin
        StatusBar1.Panels[2].Text:=rsMotor+' 2 '+rsSelected+rsBCCW;
        MotorCommand.params[0]:=5;
      end;
      if vpos=4 then begin
        StatusBar1.Panels[2].Text:=rsMotor+' 5 '+rsSelected+rsACW;
        MotorCommand.params[0]:=2;
      end;
    end else begin
      if hpos=3 then begin
        if (vpos=1) or (vpos=2) then begin
          StatusBar1.Panels[2].Text:=rsMotor+' 3'+rsSelected+rsACW;
          MotorCommand.params[0]:=4;
        end;
        if (vpos=3) or (vpos=4) then begin
          StatusBar1.Panels[2].Text:=rsMotor+' 4 '+rsSelected+rsBCCW;
          MotorCommand.params[0]:=3;
        end;
      end;
    end;
  end;

  if btnTurnAll.Enabled and UARTconnected then begin
    CreateGUI_COMMAND_LONG(msg, motorcommand);
    SendUARTMessage(msg, LengthFixPartBC);
  end;
end;

procedure TForm1.rgAddValueClick(Sender: TObject);
begin
  AddValueLineSeries1.Clear;
end;

procedure TForm1.timerGUITimer(Sender: TObject);
var
  msg: TMAVmessage;

begin
  if UARTConnected then begin
    CreateGUIheartbeat(msg);
    SendUARTMessage(msg, LengthFixPartBC);

// Send empty messages necessary? Seems not.
(*  CreateGUI_SYS_STATUS(msg);
    SendUARTMessage(msg, LengthFixPartBC);
    CreateGUIemptyMsg(msg, 32, 28);                {LOCAL_POSITION_NED}
    SendUARTMessage(msg, LengthFixPartBC);
    CreateGUIemptyMsg(msg, 30, 28);                {ATTITUDE}
    SendUARTMessage(msg, LengthFixPartBC); *)

    NumberMessagesInStatusBar;
  end;
  timerSensors.Enabled:=true;
end;

procedure TForm1.timerSensorsTimer(Sender: TObject);
begin
  ResetSensorsStatus;
end;

// Create and update charts

procedure TForm1.CreateSatSNRBarChart(const sats: TGPSdata);
var
  i, NumSatsInUse, NumSatsVisible: integer;
  IndicatorColor: TColor;

begin
  ChartSatSNR.DisableRedrawing;
  BarSatSNR.Clear;
  NumSatsInUse:=0;
  ChartSatSNR.Title.Visible:=true;
  try
    for i:=0 to MAVsatCount do begin
      if (sats.sat_used[i]<>0) then begin
        IndicatorColor:=clSatUsed;
        inc(NumSatsInUse);
      end else
        IndicatorColor:=clSatVisible;
      SatSNRBarSource.Add(i, sats.sat_snr[i], 'PRN'+IntToStr(sats.sat_prn[i]), IndicatorColor);
    end;
    NumSatsVisible:=sats.sats_visible;
    if NumSatsVisible=max8 then
      NumSatsVisible:=0;
    ChartSatSNR.Title.Text[0]:=IntToStr(NumSatsVisible)+tab1+rsVisible+tab2+
                               IntToStr(NumSatsInUse)+tab1+rsInUse;
    if (NumSatsInUse>0) and (gbPosition.Color<>clSatUsed) then
      gbPosition.Color:=clSatUsed;
    if NumSatsInUse=0 then
      gbPosition.Color:=clSensorOK;
  finally
    ChartSatSNR.EnableRedrawing;
    ChartSatSNR.Repaint;
  end;
end;

procedure TForm1.CreateSatPolarDiagram(const sats: TGPSdata);
var
  azi, ele: single;
  IndicatorColor: TColor;
  i: integer;

begin
  SatPolar.DisableRedrawing;
  try
    SatPolarSeries.Clear;
    for i:=0 to MAVsatCount do begin
      if (sats.sat_used[i]<>0) then
        IndicatorColor:=clSatUsed
      else
        IndicatorColor:=clSatVisible;
      azi:=SatAzimuthToDeg(sats.sat_azimuth[i]);
      ele:=SatElevationToDeg(sats.sat_elevation[i]);
      SatPolarSource.Add(azi, ele, IntToStr(sats.sat_prn[i]), IndicatorColor);
    end;
    PreparePolarAxes(SatPolar, 90);  {Sat elevation 0: right on top of receiver, 90: on the horizon}
  finally
    SatPolar.EnableRedrawing;
    SatPolar.Repaint;
  end;
end;


//////////////////////// Polar coordinates diagram /////////////////////////////

{https://www.lazarusforum.de/viewtopic.php?p=66139#p66139
 wp_xyz

PreparePolarAxes rufst du auf, nachdem deine Daten geladen sind und du weißt,
wie groß der maximale Radiuswert ist.
Den brauchst du als Parameter AMax in dieser Prozedur (hier fix auf 90°).
Damit werden die x- und y-Achsen auf gleichen Wertebereich eingestellt und
insgesamt komplett ausgeblendet.

DrawPolarAxes wird im OnAfterDrawBackwall-Ereignis des Charts aufgerufen.
Zu diesem Zeitpunkt sind die Daten noch nicht ausgegeben - es würde sich auch
OnAfterPaint anbieten, aber damit würden die Achsenkreise über die
Datenkurven gezeichnet und in der hier gezeigten Implementierung komplett
übermalt, weil das Hintergrundrechteck mit eingefärbt wird.
DrawPolarAxes erhält als Parameter wieder den maximalen Radius und den
Abstand der Kreise. Die komplette Zeichenausgabe ist etwas ungewohnt
("Chart.Drawer"), weil TAChart eine Zwischenschicht für die Ausgabe
eingeführt hat, so dass man verschiedene Ausgabe"geräte"
(BGRABitmap, Vektorformate, Drucker) mit demselben Code ansprechen kann.
}

procedure TForm1.PreparePolarAxes(AChart: TChart; AMax: Double);
var
  ex: TDoubleRect;

begin
  ex.a.x:= -AMax;
  ex.a.y:= -AMax;
  ex.b.x:= AMax;
  ex.b.y:= AMax;
  with AChart do begin
    Extent.FixTo(ex);
    Proportional:=true;
    Frame.Visible:=false;
    with LeftAxis do begin
      AxisPen.Visible:=false;
      Grid.Visible:=false;
      PositionUnits:=cuGraph;
      Marks.Visible:=false;
    end;
    with BottomAxis do begin
      AxisPen.Visible:=false;
      Grid.Visible:=false;
      PositionUnits:=cuGraph;
      Marks.Visible:=false;
    end;
  end;
end;

procedure TForm1.DrawPolarAxes(AChart: TChart; AMax, ADelta: Double; MeasurementUnit: string);
var
  xRadius, theta: Double;
  P1, P2: TPoint;
  i, h, w: Integer;
  AxisLabels: string;

begin
  with AChart do begin
// Background
    Drawer.SetBrushParams(bsSolid, Color);
    Drawer.FillRect(0, 0, Width, Height);

// Radial lines for direction
    Drawer.SetBrushParams(bsClear, clNone);
    Drawer.SetPenParams(psDot, clGray);
    for i:=0 to 5 do begin
      theta:=i * pi/6;
      P1:=GraphToImage(DoublePoint(AMax*sin(theta), AMax*cos(theta)));
      P2:=GraphToImage(DoublePoint(-AMax*sin(theta), -AMax*cos(theta)));
      Drawer.MoveTo(P1);
      Drawer.Lineto(P2);
    end;

// Circles
    xRadius:=ADelta;
    while xRadius <= AMax do begin
      P1:=GraphToImage(DoublePoint(-xRadius, -xRadius));
      P2:=GraphToImage(DoublePoint(+xRadius, +xRadius));
      Drawer.SetPenParams(psDot, clGray);
      Drawer.SetBrushParams(bsClear, clNone);
      Drawer.Ellipse(P1.x, P1.y, P2.x, P2.y);
      xRadius:=xRadius + ADelta;
    end;

// Axis labels
    Drawer.Font:=BottomAxis.Marks.LabelFont;
    h:=Drawer.TextExtent('0').y;
    xRadius:=0;
    while xRadius <= AMax do begin
      AxisLabels := FloatToStr(xRadius)+MeasurementUnit;
      w:=Drawer.TextExtent(AxisLabels).x;
      P1:=GraphToImage(DoublePoint(0, xRadius));
      Drawer.TextOut.Pos(P1.X - w div 2, P1.y - h div 2).Text(AxisLabels).Done;
      xRadius:=xRadius + ADelta;
    end;
  end;
end;

{DrawPolarAxes is called in the OnAfterDrawBackwall event of the chart.
 OnAfterPaint would also be an option, but this would draw the axis circles over the
 data curves because the background rectangle is also colored.}

procedure TForm1.SatPolarAfterDrawBackWall(ASender: TChart; ACanvas: TCanvas;
  const ARect: TRect);
begin
  DrawPolarAxes(ASender, 90, 15, '°');
end;

procedure TForm1.PrepareSatPolarDiagram;               {My settings for the polar diagram}
begin
  with SatPolarSeries do begin
    Source:=SatPolarSource;
    Marks.Style:=smsLabel;
    LinePen.Style:=psClear;
    ShowPoints:=true;
    Marks.LabelBrush.Color:=clPolarLabel;
    Pointer.Style:=psHexagon;
    Pointer.HorizSize:=PolarSatSize;
    Pointer.VertSize:=PolarSatSize;
    Pointer.Pen.Style:=psClear;
    Pointer.Visible:=true;
  end;
end;

procedure TForm1.PrepareSatSNRBarChart(const LeftCaption: string); {My settings for the SNR chart}
begin
  with ChartSatSNR do begin                            {for the whole chart}
    Title.Visible:=false;
    LeftAxis.Title.Caption:=LeftCaption;
    LeftAxis.Title.Visible:=true;
    BottomAxis.Marks.Source:=SatSNRBarSource;
    BottomAxis.Marks.Style:=smsLabel;
    BottomAxis.Grid.Visible:=false;
    BottomAxis.Marks.LabelFont.Orientation:=900;       {turned by 90°}
  end;

  with BarSatSNR do begin                              {For the bar serie}
    Source:=SatSNRBarSource;
    SeriesColor:=clSatUsed;
  end;
end;

end.

