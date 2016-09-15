from flask import Flask, render_template, request
import base64

app = Flask(__name__)

@app.route("/", methods=['GET', 'POST'])
def hello():
	if request.method == 'GET':
		return render_template('index.html')
	else: #Handle POST method
		f = request.form["file"]
		f = f.replace("data:audio/wav;base64,", "")
		f = base64.b64decode(f)
		with open("../babyTracker/shush.wav", "wb") as output:
			output.write(f)
		return "{ status: \"success\" }"
if __name__ == "__main__":
	context = ('ssl/ssl.cert', 'ssl/ssl.key')
	app.run(debug=True, host='0.0.0.0', ssl_context=context)
