import json
import requests
r = requests.get(url = "https://www.alphavantage.co/query?function=SMA&symbol={}&interval={}&time_period={}&series_type={}&apikey={}".format(
	"EYES",
	"daily",
	"1",
	"1",
	"close",
	"MRH0YM0UZGM5K07E"))
