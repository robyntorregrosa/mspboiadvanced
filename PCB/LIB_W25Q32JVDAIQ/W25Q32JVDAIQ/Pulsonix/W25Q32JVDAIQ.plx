PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//2008901/132563/2.38/8/0/Integrated Circuit

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c106_h66"
		(holeDiam 0.66)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 1.06) (shapeHeight 1.06))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 1.06) (shapeHeight 1.06))
	)
	(padStyleDef "s106_h66"
		(holeDiam 0.66)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.06) (shapeHeight 1.06))
		(padShape (layerNumRef 16) (padShapeType Rect)  (shapeWidth 1.06) (shapeHeight 1.06))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "DIP902W46P254L959H533Q8N" (originalName "DIP902W46P254L959H533Q8N")
		(multiLayer
			(pad (padNum 1) (padStyleRef s106_h66) (pt -4.51, 3.81) (rotation 90))
			(pad (padNum 2) (padStyleRef c106_h66) (pt -4.51, 1.27) (rotation 90))
			(pad (padNum 3) (padStyleRef c106_h66) (pt -4.51, -1.27) (rotation 90))
			(pad (padNum 4) (padStyleRef c106_h66) (pt -4.51, -3.81) (rotation 90))
			(pad (padNum 5) (padStyleRef c106_h66) (pt 4.51, -3.81) (rotation 90))
			(pad (padNum 6) (padStyleRef c106_h66) (pt 4.51, -1.27) (rotation 90))
			(pad (padNum 7) (padStyleRef c106_h66) (pt 4.51, 1.27) (rotation 90))
			(pad (padNum 8) (padStyleRef c106_h66) (pt 4.51, 3.81) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -5.545 5.33) (pt 5.545 5.33) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 5.545 5.33) (pt 5.545 -5.33) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 5.545 -5.33) (pt -5.545 -5.33) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -5.545 -5.33) (pt -5.545 5.33) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.24 5.08) (pt 3.24 5.08) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.24 5.08) (pt 3.24 -5.08) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.24 -5.08) (pt -3.24 -5.08) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.24 -5.08) (pt -3.24 5.08) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.24 3.81) (pt -1.97 5.08) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt -5.04 5.08) (pt 3.24 5.08) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.24 -5.08) (pt 3.24 -5.08) (width 0.2))
		)
	)
	(symbolDef "W25Q32JVDAIQ" (originalName "W25Q32JVDAIQ")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 5) (pt 1100 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 6) (pt 1100 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 7) (pt 1100 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 8) (pt 1100 mils -300 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -325 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 900 mils 100 mils) (width 6 mils))
		(line (pt 900 mils 100 mils) (pt 900 mils -400 mils) (width 6 mils))
		(line (pt 900 mils -400 mils) (pt 200 mils -400 mils) (width 6 mils))
		(line (pt 200 mils -400 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 950 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 950 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "W25Q32JVDAIQ" (originalName "W25Q32JVDAIQ") (compHeader (numPins 8) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "/CS") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Input))
		(compPin "2" (pinName "DO") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "3" (pinName "/WP") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "4" (pinName "GND") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Power))
		(compPin "8" (pinName "DI") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Power))
		(compPin "7" (pinName "CLK") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "6" (pinName "/HOLD") (partNum 1) (symPinNum 7) (gateEq 0) (pinEq 0) (pinType Input))
		(compPin "5" (pinName "VCC") (partNum 1) (symPinNum 8) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "W25Q32JVDAIQ"))
		(attachedPattern (patternNum 1) (patternName "DIP902W46P254L959H533Q8N")
			(numPads 8)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
				(padNum 6) (compPinRef "6")
				(padNum 7) (compPinRef "7")
				(padNum 8) (compPinRef "8")
			)
		)
		(attr "Mouser Part Number" "")
		(attr "Mouser Price/Stock" "")
		(attr "Manufacturer_Name" "Winbond")
		(attr "Manufacturer_Part_Number" "W25Q32JVDAIQ")
		(attr "Description" "NOR Flash spiFlash, 32M-bit, DTR, 4Kb Uniform Sector")
		(attr "<Hyperlink>" "https://mouser.componentsearchengine.com/Datasheets/1/W25Q32JVDAIQ.pdf")
		(attr "<Component Height>" "5.33")
		(attr "<STEP Filename>" "W25Q32JVDAIQ.stp")
	)

)