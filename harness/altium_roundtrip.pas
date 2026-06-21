{ freerouting-rs Altium round-trip harness (DelphiScript, ASCII+CRLF).
  RUN: File > Run Script (or open + F9), pick a procedure below.

  Procedures:
    ExportDsn    - export current PCB to <WORK>\board.dsn  (Specctra Design)
    ImportRte    - import <WORK>\board.rte onto current PCB (Specctra Route)
    ReportCounts - read-only: write track/via/component counts to result file

  All procedures write a log line to <WORK>\harness_log.txt so the WSL side can
  observe what happened. WORK = C:\Users\jheller2\altium_rte_test }

Const
    WORK = 'C:\Users\jheller2\altium_rte_test\';

Procedure LogLine(S : String);
Var F : TextFile;
Begin
    AssignFile(F, WORK + 'harness_log.txt');
    If FileExists(WORK + 'harness_log.txt') Then Append(F) Else Rewrite(F);
    WriteLn(F, S);
    CloseFile(F);
End;

Function CurBoard : IPCB_Board;
Begin
    Result := PCBServer.GetCurrentPCBBoard;
End;

Procedure ExportDsn;
Var B : IPCB_Board;
Begin
    B := CurBoard;
    If B = Nil Then Begin LogLine('ExportDsn: no board'); Exit; End;
    LogLine('ExportDsn: board=' + B.FileName + ' -> ' + WORK + 'board.dsn');
    { PCB:Export with the Specctra Design filter. Parameter names probed; the
      documented launch pattern is ResetParameters/AddStringParameter/RunProcess. }
    ResetParameters;
    AddStringParameter('ObjectKind', 'Specctra Design');
    AddStringParameter('FileName', WORK + 'board.dsn');
    RunProcess('PCB:Export');
    LogLine('ExportDsn: RunProcess returned. exists=' +
            BoolToStr(FileExists(WORK + 'board.dsn'), True));
    ShowMessage('ExportDsn done; see harness_log.txt');
End;

Procedure ImportRte;
Var B : IPCB_Board;
Begin
    B := CurBoard;
    If B = Nil Then Begin LogLine('ImportRte: no board'); Exit; End;
    LogLine('ImportRte: importing ' + WORK + 'board.rte onto ' + B.FileName);
    ResetParameters;
    AddStringParameter('ObjectKind', 'Specctra Route');
    AddStringParameter('FileName', WORK + 'board.rte');
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
    AssignFile(F, WORK + 'import_result.txt'); Rewrite(F);
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
