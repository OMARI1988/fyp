import random
player = 'X'
computer = 'O'


def main():
	
	a = [0,0,0]
	b = [0,0,0]
	c = [0,0,0]

	theBoard = [a,b,c]
	game = True
	while game:
		printBoard(theBoard)
		entry = raw_input("Enter coords 'XY' (X = column, Y = row): ") 
		theBoard = makeMove(theBoard,(int(entry[1]),int(entry[0])),player)
		if isWinner(theBoard, player):
			printBoard(theBoard)
			print('Hooray! You have won the game!')
			break
		elif isBoardFull(theBoard): 
			printBoard(theBoard)
			print('The game is a tie!')
			break
			
		entryComp = getComputerMove(theBoard)
		theBoard = makeMove(theBoard,entryComp,computer)
		
		if isWinner(theBoard, computer):
			printBoard(theBoard)
			print('The computer has beaten you! You lose.')
			game = False
		elif isBoardFull(theBoard):
			printBoard(theBoard)
			print('The game is a tie!')
			break
    

def chooseRandomMove(board, movesList):
    # Returns a valid move from the passed list on the passed board.
    # Returns None if there is no valid move.
    possibleMoves = []
    for i in movesList:
        if not board[i[0]][i[1]]:
        		
            possibleMoves.append(i)

    if len(possibleMoves) != 0:
        return random.choice(possibleMoves)
    else:
        return None

def getComputerMove(board):

	# check to see if winning move is possible
	for row in range(len(board)):
		for space in range(len(board[row])):
			copy = getBoardCopy(board)
			if  not copy[row][space]: # if space is free 
				makeMove(copy,(row,space), computer)
				if isWinner(copy, computer):
						return (row,space)
						
	# Check if the player could win on his next move, and block them.
	for row in range(len(board)):
		for space in range(len(board[row])):
			copy = getBoardCopy(board)
			if  not copy[row][space]: # if space is free 
				makeMove(copy, (row,space), player)
				if isWinner(copy, player):
					return (row,space)
  
	# Try to take one of the corners, if they are free.
	move = chooseRandomMove(board, [(0,0),(2,0),(0,2),(2,2)])
	if move != None:
		return move     
		         
		         
	# Try to take the center, if it is free.
	if not board[1][1]:
		return (1,1)
				
	# Move on one of the sides.  
	return chooseRandomMove(board, [(0,1), (1,0), (2,1), (1,2)])
						
def getBoardCopy(board):
	dupeBoard = []
	for row in range(len(board)):
		rowtemp = []
		for space in range(len(board[row])):
			rowtemp.append(board[row][space])
		dupeBoard.append(rowtemp)

	return dupeBoard
			
			
def isBoardFull(board):
    # Return True if every space on the board has been taken. Otherwise return False.
    for row in range(len(board)):
			for space in range(len(board[row])):
				if  not board[row][space]: # if space is free 
					return False
    
    return True

def makeMove(board, entry, letter):
		board[entry[0]][entry[1]] = letter
		
		return board	

def isWinner(bo, le):
		return ((bo[0][0] == le and bo[0][1] == le and bo[0][2] == le) or # across the top
		(bo[1][0] == le and bo[1][1] == le and bo[1][2] == le) or # across the middle
		(bo[2][0] == le and bo[2][1] == le and bo[2][2] == le) or # across the bottom
		(bo[0][0] == le and bo[1][0] == le and bo[2][0] == le) or # down the left
		(bo[0][1] == le and bo[1][1] == le and bo[2][1] == le) or # down the middle
		(bo[0][2] == le and bo[1][2] == le and bo[2][2] == le) or # down the right
		(bo[0][0] == le and bo[1][1] == le and bo[2][2] == le) or # diag 1 
		(bo[0][2] == le and bo[1][1] == le and bo[2][0] == le))		# diag 2
		
def printBoard(board):
	counter = 0
	print "  0 1 2"
	for row in range(len(board)):
		print " -------"
		string = str(counter)
		for space in range(len(board[row])):
			string += "|"
			if not board[row][space]:
				string += " "
			else:
				string += str(board[row][space])
		print string + "|"
		counter += 1  
	print " -------"
	
  
  
if __name__ == "__main__":
	main()