
function randint(max){
	  return Math.floor(Math.random()*max);
}

letters = "QWERTYUIOPASDFGHJKLZXCVBNM".split("")

for(let i=0; i<100; i++){
	  if(randint(10) > 5)
		    plate = randint(10) + "" + randint(10) + "" + randint(10) + " " + letters[randint(26)] + letters[randint(26)] + letters[randint(26)] + letters[randint(26)];
	  else
		    plate = letters[randint(26)] + letters[randint(26)] + letters[randint(26)] + letters[randint(26)] + " " + randint(10) + "" + randint(10) + "" + randint(10);

	  console.log(plate);
}
