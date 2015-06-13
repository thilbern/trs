function [  ] =bagAofFeatureTest(img)

rootFolder = fullfile('youbot', 'pictures'); % define output folder

imgSets = [ imageSet(fullfile(rootFolder, 'dog.png')), ...
            imageSet(fullfile(rootFolder, 'plant.png')), ...
            imageSet(fullfile(rootFolder, 'pumpkin.png')), ...
            imageSet(fullfile(rootFolder, 'trashcan.png')), ...
            imageSet(fullfile(rootFolder, 'trike.png')) ];
            bag = bagOfFeatures(imgSets,'Verbose',false);
            
            %featureVector = encode(bag, img);
            
categoryClassifier = trainImageCategoryClassifier(imgSets, bag);

[labelIdx, scores] = predict(categoryClassifier, img);

% Display the string label
categoryClassifier.Labels(labelIdx)


end
