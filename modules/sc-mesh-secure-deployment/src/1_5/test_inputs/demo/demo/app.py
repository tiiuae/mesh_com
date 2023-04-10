from flask import Flask, render_template, jsonify, request
import pandas as pd
import json
import os

app = Flask(__name__)

app.config['JSON_SORT_KEYS'] = False

project_dir = os.path.dirname(os.path.abspath(__file__))
my_files = r'/static/data/'
file_dir = project_dir + my_files


@app.route('/')
def index():

#    json_file = file_dir + r'data_set_1.json'

#    with open(json_file) as f:
#        js_object = json.load(f)
	#        df = pd.read_json(json.dumps(js_object))
     df = pd.read_csv('global_table.csv')
     return render_template('index.html', data=df)


@app.route('/get_data', methods=['POST'])
def get_data_function():

    user = request.form['user']

    if user == 'two':

        json_file = file_dir + r'data_set_2.json'

        with open(json_file) as f:
            js_object = json.load(f)
            return jsonify(js_object)

    else:
        json_file = file_dir + r'data_set_1.json'

        with open(json_file) as f:
            js_object = json.load(f)
            return jsonify(js_object)


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
