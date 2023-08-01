# This function test the pdf generation module inside this conversion pipeline.
# This function use the plots inside the ./tests/pdf_generation folder as resources
# Author: Huawei Wang, June 22, 2023

from myoconverter.utils.generate_pdf import generate_pdf

cvt1_path = './myoconverter/tests/resource/generate_pdf_test/Step1_xmlConvert'
cvt2_path = './myoconverter/tests/resource/generate_pdf_test/Step2_muscleKinematics'
cvt3_path = './myoconverter/tests/resource/generate_pdf_test/Step3_muscleKinetics'
model_name = 'test'
saving_path = './myoconverter/tests/resource/generate_pdf_test/'

generate_pdf(cvt1_path, cvt2_path, cvt3_path, model_name, saving_path)