import random
player = 'X'
computer = 'O'



def allEqual(x,y,z):

	return (x==y) and (y==z)


def isSorted(x,y,z):
    
    return (x > y and y > z) or (z > y and y > x)
	
	
def main():
    a = [0,0,0]
    b = [0,0,0]
    c = [0,0,0]
    d = [0,0,0]
    e = [0,0,0]
    f = [0,0,0]
    g = [0,0,0]
    h = [0,0,0]
    i = [0,0,0]

    level1 = [a,b,c]
    level2 = [d,e,f]
    level3 = [g,h,i]

    theBoard = [level1,level2,level3]
    
    spaces = []
    for i in range(3):
        for j in range(3):
            for k in range(3):
                spaces.append((i,j,k))


    counter = 0
    solutions = []
    for a in spaces:
        for b in spaces:
            for c in spaces:	 
                if a != b and b != c and a != c:
                    line = [a,b,c]
                    line.sort()
                    if not line in solutions:
                        level_eq = allEqual(a[0],b[0],c[0])
                        row_eq = allEqual(a[1],b[1],c[1])
                        col_eq = allEqual(a[2],b[2],c[2])
                        level_sort = isSorted(a[0],b[0],c[0])
                        row_sort = isSorted(a[1],b[1],c[1])
                        col_sort = isSorted(a[2],b[2],c[2])
                        if ((level_eq and row_eq) or (level_eq and col_eq) or (col_eq and row_eq)) or \
                        (level_sort and row_sort and col_sort) or \
                        ((level_eq and row_sort and col_sort) or (level_sort and row_eq and col_sort) or (level_sort and row_sort and col_eq)):
                            solutions.append(line)
                            
    #print solutions, len(solutions)
	#print counter		
			
	#printBoard(theBoard)
    game = True
    while game:
		printBoard(theBoard)
		entry = raw_input("Enter coords 'XY' (X = row, Y = column): ") 
		theBoard = makeMove(theBoard,(int(entry[0]),int(entry[1])),player)
		if isWinner(theBoard, solutions, player):
			printBoard(theBoard)
			print('Hooray! You have won the game!')
			break
		elif isBoardFull(theBoard): 
			printBoard(theBoard)
			print('The game is a tie!')
			break
			
		entryComp = getComputerMove(theBoard,solutions)
		theBoard = makeMove(theBoard,entryComp,computer)
		
		if isWinner(theBoard, solutions, computer):
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
        if not board[len(board)-1][i[0]][i[1]]:
        		
            possibleMoves.append(i)

    if len(possibleMoves) != 0:
        return random.choice(possibleMoves)
    else:
        return None

def getComputerMove(board,solutions):

	# check to see if winning move is possible
	for row in range(len(board[0])):
		for space in range(len(board[0][row])):
			for level in range(len(board)):
			    copy = getBoardCopy(board)
			    if  not copy[level][row][space]: # if space is free 
				    makeMove(copy,(row,space), computer)
				    if isWinner(copy, solutions, computer):
						    return (row,space)

		        
						
	# Check if the player could win on his next move, and block them.
	for row in range(len(board[0])):
		for space in range(len(board[0][row])):
			for level in range(len(board)):
			    copy = getBoardCopy(board)
			    if  not copy[level][row][space]: # if space is free 
				    makeMove(copy, (row,space), player)
				    if isWinner(copy, solutions, player):
					    return (row,space)
  
	# Try to take one of the corners, if they are free.
	move = chooseRandomMove(board, [(0,0),(2,0),(0,2),(2,2)])
	if move != None:
		return move     
		         
		         
	# Try to take the center, if it is free.
	if not board[len(board)-1][1][1]:
		return (1,1)
				
	# Move on one of the sides.  
	return chooseRandomMove(board, [(0,1), (1,0), (2,1), (1,2)])
						
def getBoardCopy(board):
    dupeBoard = []
    for level in range(len(board)):
        leveltemp = []
        for row in range(len(board[level])):
            rowtemp = []
            for space in range(len(board[level][row])):
                rowtemp.append(board[level][row][space])
            leveltemp.append(rowtemp)
        dupeBoard.append(leveltemp)

    return dupeBoard
			
			
def isBoardFull(board):
    # Return True if every space on the top of the board has been taken. Otherwise return False.
    for row in range(len(board[len(board)-1])):
			for space in range(len(board[2][row])):
				if  not board[2][row][space]: # if space is free 
					return False
    
    return True

def makeMove(board, entry, letter):
		
    for level in range(len(board)):
        if not board[level][entry[0]][entry[1]]:
            board[level][entry[0]][entry[1]] = letter
            break
	        
    return board	

def isWinner(board, solutions, letter):
		for solution in solutions:
		    if board[solution[0][0]][solution[0][1]][solution[0][2]] == letter \
		     and board[solution[1][0]][solution[1][1]][solution[1][2]] == letter \
		     and board[solution[2][0]][solution[2][1]][solution[2][2]] == letter:
		            return True
		
def printBoard(board):
	for level in range(len(board)-1,-1,-1):
		counter = 0
		print "  0 1 2"
		for row in range(len(board[level])):
			print " -------"
			string = str(counter)
			for space in range(len(board[level][row])):
				string += "|"
				if not board[level][row][space]:
					string += " "
				else:
					string += str(board[level][row][space])
			print string + "|"
			counter += 1  
		print " -------"
	
  
  
if __name__ == "__main__":
	main()
