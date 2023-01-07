#include <MsgBoxConstants.au3>
#include <GUIConstants.au3>
#include <Color.au3>

Local $ColorPosX = @DesktopWidth / 2 - 5

GUICreate("", 20,20,$ColorPosX,10,$WS_POPUP,$WS_EX_TOPMOST)
GUISetBkColor(0x00FF00)
GUISetState()

; 10x10 크기의 영역에 대해서 색깔 평균값 구하기.
Func GetRangeColor($posX, $posY)
	Local $range = 10
	If $posX + $range >= @DesktopWidth Then
		$posX = @DesktopWidth - $range
	EndIf
	If $posY + $range >= @DesktopHeight Then
		$posY = @DesktopHeight - $range
	EndIf
	Local $count = 0
	Local $nAccR = 0
	Local $nAccG = 0
	Local $nAccB = 0

	For $x = $posX To $posX + $range Step 1
		For $y = $posY To $posY + $range Step 1
			Local $aColor = _ColorGetRGB(PixelGetColor($x,$y))
			$nAccR += $aColor[0];
			$nAccG += $aColor[1];
			$nAccB += $aColor[2];
			$count += 1
		Next
	Next
	Local $aRGB[3]
	$aRGB[0] = $nAccR / $count
	$aRGB[1] = $nAccG / $count
	$aRGB[2] = $nAccB / $count
	;Local $aRGB = {$valR / $count, $valG / $count, $valB / $count}
	Local $result = _ColorSetRGB($aRGB)
	return $result
EndFunc   ;==>Today

While (true)
	Local $aPos = MouseGetPos()
	;Local $iColor = PixelGetColor($aPos[0], $aPos[1])
	;Local $iColor = GetRangeColor($aPos[0], $aPos[1])
	Local $iColor = GetRangeColor($ColorPosX + 20, 20 + 20)
	GUISetBkColor($iColor)
	Sleep(300)
WEnd

#comments-start
Sleep(500)
MsgBox($MB_SYSTEMMODAL, "", "The hex color is: " & Hex($iColor, 6))

Local $aCoord = PixelSearch(0, 0, @DesktopWidth, @DesktopHeight, $iColor)
If Not @error Then
    MsgBox($MB_SYSTEMMODAL, "", "X and Y are: " & $aCoord[0] & "," & $aCoord[1])
EndIf


;MouseMove($aCoord[0], $aCoord[1], 0)

Local $aArray[0]
For $x = 0 To @DesktopWidth Step 1
   For $y = 0 To @DesktopHeight Step 1
      If PixelGetColor($x,$y) = $iColor Then
         ;Add $x and $y to Array using _ArrayAdd() (or whatever you prefer)
         MouseMove($x, $y, 10)
         _ArrayAdd($aArray, $x & "-" & $y)
      EndIf
   Next
Next
_ArrayDisplay($aArray, "Pixels")
#comments-end

