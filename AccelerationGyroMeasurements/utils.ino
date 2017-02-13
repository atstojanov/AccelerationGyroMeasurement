// Помощни функции

// Изчисляване на позицията на редовете на екрана
int rowPos(int row)
{
	return row * FONT_HEIGHT;
}

// Изчисляване на позицията на колоните на екрана
int colPos(int col)
{
	return col * FONT_WIDTH;
}

// Визуализация на число с плаваща запетая на дисплея
void displayNumberF(float v, int col, int row, int length)
{
	if (v < 0)
	{
		display.print(F("-"), colPos(col), rowPos(row));
	}
	else
	{
		display.print(F("+"), colPos(col), rowPos(row));
	}
	display.printNumF(abs(v), 2, colPos(col + 1), rowPos(row), '.', length, '0');
}

// Визуализация на +- знака на дисплея
void displayPlusMinus(int col, int row)
{
	display.drawBitmap(colPos(col), rowPos(row), plusminus, 6, 8);
}

// Визуализация на обхвата на акселерометъра
void displayAccRange(int col, int row)
{
	displayPlusMinus(col, row);
	displayText(accRanges[accRange], col + 1, row);
}

// Визуализация на имената на сензорите
void displaySensorName(int col, int row)
{
	switch (getSensorID())
	{
	case MPU6050_ID:
		displayText(F("MPU6050"), col, row);
		break;
	case ADXL345_ID:
		displayText(F("ADXL345"), col, row);
		break;
	}
}

// Визуализация на текст на дисплея, прочетен от програмната памет 
void displayText(const __FlashStringHelper *st, int col, int row)
{
	char buffer[strlen_P((PGM_P)st) + 1];

	strcpy_P(buffer, (PGM_P)st);
	displayText(buffer, col, row);
}

// Визуализация на текст на дисплея
void displayText(char *st, int col, int row)
{
	display.print(st, colPos(col), rowPos(row));
}

// Визуализация на текст на дисплея
void displayText(String st, int col, int row)
{
	display.print(st, colPos(col), rowPos(row));
}