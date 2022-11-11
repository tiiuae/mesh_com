from flask import Flask, request, json
import pandas as pd
app = Flask(__name__)


@app.route('/DE')
def debug2():
    try:
        table = pd.read_csv("auth/decision_table.csv")
        table['MAC Address'] = table.index
        table.reset_index(drop=True, inplace=True)
        bes = table.to_html(classes='table table-striped', header=True, index=False)
        information = pd.read_csv("common/information.csv")
        inf_html = information.to_html(classes='table table-striped', header=True, index=False)

        return (
            f'<h3>Reference</h3>{inf_html}'
            f'<h3>Table with Decision</h3>{bes}'

            )
    except FileNotFoundError:
        return (
            f'<h3>SecTable not found</h3>'
        )



@app.route('/')
def debug():
    try:
        table = pd.read_csv("auth/dev.csv")
        table['MAC Address'] = table.index
        table.reset_index(drop=True, inplace=True)
        bes = table.to_html(classes='table table-striped', header=True, index=False)


        return (
                f'<h3>SecTable</h3>{bes}'
            )
    except FileNotFoundError:
        return (
            f'<h3>SecTable not found</h3>'
        )


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
