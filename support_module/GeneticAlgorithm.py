from __future__ import annotations

from random import randint

class Gene:
    def __init__(self, gene) -> None:
        self.gene = gene

class Chromosome:
    def __init__(self, genes: list[Gene]) -> None:
        self.genes = genes

        self.fitness: float = 0

class GeneticAlgorithm:
    def __init__(self, population_size: int) -> None:
        self.population_size = population_size

        self.population: list[Chromosome] = []
        self.generation = 0

    def initalize_population(
        self,
        randomizers: list[tuple[callable, tuple]]
    ) -> list[Chromosome]:
        for _ in range(self.population_size):
            genes: list[Gene] = []
            for randomizer, r_range in randomizers:
                genes.append(Gene(
                    gene=randomizer() * (r_range[1] - r_range[0]) + r_range[0]
                ))
            self.population.append(Chromosome(genes))

    def top_percent(self, percent: float) -> list[Chromosome]:
        self.population.sort(key=lambda x: x.fitness, reverse=True)
        return self.population[:int(len(self.population) * percent)]

    def crossover(self, parent1: Chromosome, parent2: Chromosome, split: int) -> tuple(Chromosome):
        child_1 = parent1.genes[:split] + parent2.genes[split:]
        child_2 = parent2.genes[:split] + parent1.genes[split:]
        return child_1, child_2

    def mutate_population(self, percent: float):
        self.population: list[Chromosome] = self.top_percent(percent)
        for i in range(0, len(self.population), 2):
            parent_1 = self.population[i]
            parent_2 = self.population[i + 1]
            child_1, child_2 = self.crossover(
                parent_1, parent_2, randint(1, len(parent_1.genes)-2)
            )

            self.population.append(Chromosome(child_1))
            self.population.append(Chromosome(child_2))