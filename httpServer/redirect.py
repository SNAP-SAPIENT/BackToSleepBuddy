from flask import Flask, request, redirect

app = Flask(__name__)

@app.route("/", methods=['GET', 'POST'])
def hello():
	if request.url.startswith('http://'):
		return redirect(request.url.replace('http', 'https', 1).replace('80', '443', 1))
	else:
		return "blank"
	
if __name__ == "__main__":
	app.run(debug=True, host='0.0.0.0', port=80)
