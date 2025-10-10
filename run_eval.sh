!#/bin/bash
venv venv 
source venv/bin/activate
pip install -r requirements.txt

cd src/Evaluation/
python3 eval.py
