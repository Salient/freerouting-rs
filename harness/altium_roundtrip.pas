{ freerouting-rs Altium round-trip harness (DelphiScript, ASCII+CRLF).
  RUN: File > Run Script (or open + F9), pick a procedure below.

  Procedures:
    ExportDsn    - export current PCB to <WORK>\board.dsn  (Specctra Design)
    ImportRte    - import <WORK>\board.rte onto current PCB (Specctra Route)
    ReportCounts - read-only: write track/via/component counts to result file
    DrcReport    - run Altium's design-rule check and write the violation count
    RoundTrip    - ImportRte then DrcReport (full import-and-verify in one click)

  All procedures write a log line to <WORK>\harness_log.txt so the WSL side can
  observe what happened.

  WORK is taken from the FREEROUTING_WORK environment variable if set (so this
  script is machine-agnostic); otherwise it falls back to a default path. Set it
  on Windows with:  setx FREEROUTING_WORK "C:\path\to\altium_rte_test\"  }

Function WorkDir : String;
Begin
    Result := GetEnvironmentVariable('FREEROUTING_WORK');
    If Result = '' Then Result := 'C:\altium_rte_test\';
    { ensure a trailing backslash }
    If Copy(Result, Length(Result), 1) <> '\' Then Result := Result + '\';
End;

Procedure LogLine(S : String);
Var F : TextFile; W : String;
Begin
    W := WorkDir;
    AssignFile(F, W + 'harness_log.txt');
    If FileExists(W + 'harness_log.txt') Then Append(F) Else Rewrite(F);
    WriteLn(F, S);
    CloseFile(F);
End;

Function CurBoard : IPCB_Board;
Begin
    Result := PCBServer.GetCurrentPCBBoard;
End;

Procedure ExportDsn;
Var B : IPCB_Board; W : String;
Begin
    B := CurBoard; W := WorkDir;
    If B = Nil Then Begin LogLine('ExportDsn: no board'); Exit; End;
    LogLine('ExportDsn: board=' + B.FileName + ' -> ' + W + 'board.dsn');
    { PCB:Export with the Specctra Design filter. Parameter names probed; the
      documented launch pattern is ResetParameters/AddStringParameter/RunProcess. }
    ResetParameters;
    AddStringParameter('ObjectKind', 'Specctra Design');
    AddStringParameter('FileName', W + 'board.dsn');
    RunProcess('PCB:Export');
    LogLine('ExportDsn: RunProcess returned. exists=' +
            BoolToStr(FileExists(W + 'board.dsn'), True));
    ShowMessage('ExportDsn done; see harness_log.txt');
End;

Procedure ImportRte;
Var B : IPCB_Board; W : String;
Begin
    B := CurBoard; W := WorkDir;
    If B = Nil Then Begin LogLine('ImportRte: no board'); Exit; End;
    LogLine('ImportRte: importing ' + W + 'board.rte onto ' + B.FileName);
    ResetParameters;
    AddStringParameter('ObjectKind', 'Specctra Route');
    AddStringParameter('FileName', W + 'board.rte');
    RunProcess('PCB:Import');
    LogLine('ImportRte: RunProcess returned.');
    ReportCounts;
    ShowMessage('ImportRte done; see import_result.txt');
End;

Procedure ReportCounts;
Var
    B : IPCB_Board; It : IPCB_BoardIterator; O : IPCB_Primitive;
    T,V,C : Integer; F : TextFile;
Begin
    B := CurBoard;
    AssignFile(F, WorkDir + 'import_result.txt'); Rewrite(F);
    If B = Nil Then Begin WriteLn(F,'no board'); CloseFile(F); Exit; End;
    T:=0; V:=0; C:=0;
    It := B.BoardIterator_Create;
    It.AddFilter_ObjectSet(MkSet(eTrackObject, eViaObject, eComponentObject));
    It.AddFilter_LayerSet(AllLayers);
    It.AddFilter_Method(eProcessAll);
    O := It.FirstPCBObject;
    While O <> Nil Do
    Begin
        If O.ObjectId = eTrackObject     Then T := T + 1;
        If O.ObjectId = eViaObject       Then V := V + 1;
        If O.ObjectId = eComponentObject Then C := C + 1;
        O := It.NextPCBObject;
    End;
    B.BoardIterator_Destroy(It);
    WriteLn(F, 'COMPONENTS=' + IntToStr(C));
    WriteLn(F, 'TRACKS=' + IntToStr(T));
    WriteLn(F, 'VIAS=' + IntToStr(V));
    CloseFile(F);
    LogLine('ReportCounts: C=' + IntToStr(C) + ' T=' + IntToStr(T) + ' V=' + IntToStr(V));
End;

{ Run Altium's design-rule check and write the violation count — closes the loop
  on "is the imported route DRC-clean in the real tool?" }
Procedure DrcReport;
Var
    B : IPCB_Board; It : IPCB_BoardIterator; O : IPCB_Primitive;
    Viol : Integer; F : TextFile;
Begin
    B := CurBoard;
    AssignFile(F, WorkDir + 'drc_result.txt'); Rewrite(F);
    If B = Nil Then Begin WriteLn(F,'no board'); CloseFile(F); Exit; End;
    { trigger a board-wide DRC so violation objects exist, then count them. }
    ResetParameters;
    AddStringParameter('Action', 'Run');
    RunProcess('PCB:DesignRuleCheck');
    Viol := 0;
    It := B.BoardIterator_Create;
    It.AddFilter_ObjectSet(MkSet(eViolationObject));
    It.AddFilter_LayerSet(AllLayers);
    It.AddFilter_Method(eProcessAll);
    O := It.FirstPCBObject;
    While O <> Nil Do Begin Viol := Viol + 1; O := It.NextPCBObject; End;
    B.BoardIterator_Destroy(It);
    WriteLn(F, 'VIOLATIONS=' + IntToStr(Viol));
    CloseFile(F);
    LogLine('DrcReport: violations=' + IntToStr(Viol));
    ShowMessage('DRC done; violations=' + IntToStr(Viol) + ' (see drc_result.txt)');
End;

{ Full import-and-verify in one click: import the .rte, count items, run DRC. }
Procedure RoundTrip;
Begin
    ImportRte;
    DrcReport;
End;
