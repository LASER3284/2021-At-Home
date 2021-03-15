Option Explicit

Const ForReading = 1
Const ForWriting = 2

Dim strComputer, varOriginalFile, objFile1, objFSO, var1, varOutFile, outfile
Dim s, v, a, x, y, H, hh, varSplit, varFirstRun, inFolder, outFolder, oFile

strComputer = "."
inFolder = "D:\workspace\2021-At-Home\2021-At-Home\PathPlanner\paths\"
'inFolder = "C:\PathPlanner\inpaths\"
outFolder = "D:\workspace\2021-At-Home\2021-At-Home\src\main\deploy\paths\output\"
'outFolder = "C:\PathPlanner\paths\"

Set objFSO = CreateObject("Scripting.FileSystemObject")

For Each oFile In objFSO.GetFolder(inFolder).Files
  If UCase(objFSO.GetExtensionName(oFile.Name)) = "CSV" Then
	'wscript.echo oFile.Name

varOutFile = outFolder & Replace(oFile.Name, ".csv", ".wpilib.json", 1)
varOriginalFile = oFile
varFirstRun = "Yes"

Set outfile = objFSO.OpenTextFile(varOutFile, ForWriting, True)

outfile.writeline "["

Set objFile1 = objFSO.OpenTextFile(varOriginalFile, ForReading)
	While Not objFile1.AtEndOfStream
		var1 = objFile1.ReadLine
		'outfile.writeline var1
		varSplit = Split(var1, ",")
		'wscript.echo varSplit
		s = varSplit(0)
		v = varSplit(1)
		a = varSplit(2)
		x = varSplit(3)
		y = -varSplit(4)
		H = -varSplit(5)
		hh = -varSplit(6)

		'Count = ubound(varSplit)
		If varFirstRun = "Yes" Then
			'Do Nothing first time
			varFirstRun = "No"
		else
			outfile.writeline "},"
		End If
		outfile.writeline "{"
		outfile.writeline chr(34) & "time" & chr(34) & ":" & s & ","
		outfile.writeline chr(34) & "velocity" & chr(34) & ":" & v & ","
		outfile.writeline chr(34) & "acceleration" & chr(34) & ":" & a & ","
		outfile.writeline chr(34) & "pose" & chr(34) & ":"
		outfile.writeline "{"
			outfile.writeline chr(34) & "translation" & chr(34) & ":"
			outfile.writeline "{"
				outfile.writeline chr(34) & "x" & chr(34) & ":" & x & ","
				outfile.writeline chr(34) & "y" & chr(34) & ":" & y
			outfile.writeline "},"
			outfile.writeline chr(34) & "rotation" & chr(34) & ":"
			outfile.writeline "{"
				outfile.writeline chr(34) & "radians" & chr(34) & ":" & H
			outfile.writeline "}"
		outfile.writeline "},"
			outfile.writeline chr(34) & "curvature" & chr(34) & ":" & hh
		'outfile.writeline "},"
	Wend

outfile.writeline "}"
outfile.writeline "]"

outfile.Close
objFile1.Close

  End if
Next

Set objFSO = Nothing
wscript.echo "Done"
WScript.Quit
