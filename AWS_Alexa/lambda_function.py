import json
import os
import boto3

# Name of the SQS queue 
# Could also store this in an environment variable and retrieve with
queue_name = os.environ['SQS']
#queue_name = 'RobotOfThings'

def build_response(message, session_attributes={}):
    response = {}
    response['version'] = '1.0'
    response['sessionAttributes'] = session_attributes
    response['response'] = {'outputSpeech':message}
    return response
    
def build_PlainSpeech(body):
    speech = {}
    speech['type'] = 'PlainText'
    speech['text'] = body
    return speech
    
def lambda_handler(event, context):
    # DEBUG - Set Environment variable debuge to True
    if os.environ['DEBUG'] == "True":
        print(json.dumps(event))

    # Build the command
    intent = event['request']['intent']
    command = json.loads('{ "messageFormat": "PlainText","dialogState": "Fulfilled","slots":[],"intentName" : "'+intent['name']+'" }')
    
    if 'slots' in intent:
        command['slots'] = json.loads('[]')
        slots = command['slots']
        for attribute in intent['slots']:
            jsonvalue = intent['slots'][attribute]
            value = jsonvalue['value']
            # if there is a resolution update the value
            if 'resolutions' in jsonvalue:
                resolutions = jsonvalue['resolutions']['resolutionsPerAuthority']
                values = resolutions[0]['values']
                value = values[0]['value']['name']
            slots.append({ attribute: value })
            
    # Send Message to RobotOfThings SQS
    sqs = boto3.resource('sqs')
    queue = sqs.get_queue_by_name(QueueName=queue_name)
    result = queue.send_message(MessageBody=json.dumps(command))

    message = build_PlainSpeech("OK")
    return build_response(message)
