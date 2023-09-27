.DEFAULT_GOAL := check

check:
	pre-commit run -a

changelog:
	cz bump --changelog
