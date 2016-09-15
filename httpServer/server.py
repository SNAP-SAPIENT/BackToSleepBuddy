from flask import Flask, render_template

app = Flask(__name__)

@app.route("/")
def hello():
	return render_template('index.html')

if __name__ == "__main__":
	context = ('ssl/ssl.cert', 'ssl/ssl.key')
	app.run(debug=False, host='0.0.0.0', ssl_context=context)
