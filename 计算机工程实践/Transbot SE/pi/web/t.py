from google import genai

client = genai.Client(api_key="AIzaSyA_bIYCfw1IGZoYn1RitnDcIby3Iv3qhM8")

response = client.models.generate_content(
    model="gemini-2.5-flash", contents="Explain how AI works in a few words"
)
print(response.text)